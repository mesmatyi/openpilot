import io
import requests
import runpy
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from openpilot.system.loggerd.xattr_cache import getxattr, setxattr


class TestBenchUploader(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.root = Path(self.temp.name)
    params = Mock()
    params.get.return_value = None
    params.get_bool.return_value = False
    self.api = Mock()
    self.api.get.return_value = SimpleNamespace(status_code=200, text='{"url": "https://upload.invalid/file", "headers": {}}')
    messaging = Mock()
    self.stream = Mock(side_effect=lambda *args: (io.BytesIO(b"recorded data"), 13))
    modules = {
      "requests": requests,
      "openpilot.cereal": SimpleNamespace(log=Mock(), messaging=messaging),
      "openpilot.cereal.messaging": messaging,
      "openpilot.common.api": SimpleNamespace(Api=lambda dongle: self.api),
      "openpilot.common.utils": SimpleNamespace(get_upload_stream=self.stream),
      "openpilot.common.params": SimpleNamespace(Params=lambda: params),
      "openpilot.common.realtime": SimpleNamespace(set_core_affinity=Mock()),
      "openpilot.common.hardware.hw": SimpleNamespace(Paths=Mock()),
      "openpilot.common.swaglog": SimpleNamespace(cloudlog=Mock()),
    }
    with patch.dict("sys.modules", modules):
      self.module = runpy.run_path(str(Path(__file__).parents[1] / "uploader.py"))
    self.uploader = self.module["Uploader"]("test-device", str(self.root))
    self.globals = self.uploader.upload.__globals__
    self.globals["fake_upload"] = False

  def make_segment(self, name="00000001--0123456789--0", bench=True):
    segment = self.root / name
    segment.mkdir()
    if bench:
      setxattr(str(segment), self.module["BENCH_ATTR_NAME"], self.module["BENCH_ATTR_VALUE"])
    return segment

  def make_file(self, segment, name):
    path = segment / name
    path.write_bytes(b"recorded data")
    return path

  def complete(self, path):
    setxattr(str(path), self.module["UPLOAD_ATTR_NAME"], self.module["UPLOAD_ATTR_VALUE"])

  def test_all_recorded_files_selected_after_stop_and_restart(self):
    segment = self.make_segment()
    expected = self.module["BENCH_UPLOAD_FILES"]
    for name in expected:
      self.make_file(segment, name)
    # Current BenchMode is never consulted: directory identity survives stop/reboot.
    uploaded = []
    for _ in expected:
      candidate = self.uploader.next_file_to_upload(metered=False)
      self.assertIsNotNone(candidate)
      name, _, filename = candidate
      uploaded.append(name)
      self.complete(filename)
      self.uploader = self.module["Uploader"]("test-device", str(self.root))
    self.assertEqual(set(uploaded), expected)
    self.assertIn(uploaded[0], ("qlog", "qlog.zst"))
    self.assertEqual(uploaded[2], "qcamera.ts")
    self.assertIsNone(self.uploader.next_file_to_upload(metered=False))

  def test_non_bench_and_unrecognized_files_are_not_opted_in(self):
    regular = self.make_segment(bench=False)
    self.make_file(regular, "rlog.zst")
    self.make_file(regular, "fcamera.hevc")
    bench = self.make_segment("00000002--0123456789--0")
    self.make_file(bench, "private.txt")
    self.make_file(bench, "rlog.zst.lock")
    self.assertIsNone(self.uploader.next_file_to_upload(metered=False))
    (bench / "rlog.zst.lock").unlink()
    self.assertIsNone(self.uploader.next_file_to_upload(metered=False))
    # A wrong marker value must not opt a regular drive into full uploads.
    setxattr(str(regular), self.module["BENCH_ATTR_NAME"], b"0")
    self.assertIsNone(self.uploader.next_file_to_upload(metered=False))

  def test_metered_network_keeps_existing_lightweight_policy(self):
    segment = self.make_segment()
    for name in self.module["BENCH_UPLOAD_FILES"]:
      self.make_file(segment, name)
    for _ in range(2):
      name, _, filename = self.uploader.next_file_to_upload(metered=True)
      self.assertIn(name, ("qlog", "qlog.zst"))
      self.complete(filename)
    self.assertIsNone(self.uploader.next_file_to_upload(metered=True))
    self.assertIsNotNone(self.uploader.next_file_to_upload(metered=False))

  def test_locks_and_uploaded_attributes_still_apply(self):
    segment = self.make_segment()
    recording = self.make_file(segment, "fcamera.hevc")
    lock = self.make_file(segment, "rlog.lock")
    self.assertIsNone(self.uploader.next_file_to_upload(metered=False))
    lock.unlink()
    self.assertEqual(self.uploader.next_file_to_upload(False)[2], str(recording))
    self.complete(recording)
    self.assertIsNone(self.uploader.next_file_to_upload(False))

  def test_bench_selection_rejects_symlinks_and_directories(self):
    segment = self.make_segment()
    source = self.root / "unrelated"
    source.write_bytes(b"not a recording")
    (segment / "fcamera.hevc").symlink_to(source)
    (segment / "rlog.zst").mkdir()
    self.assertIsNone(self.uploader.next_file_to_upload(False))

  def test_existing_boot_and_qlog_upload_priority_is_preserved(self):
    bench = self.make_segment()
    self.make_file(bench, "rlog.zst")
    normal = self.make_segment("00000002--0123456789--0", bench=False)
    qlog = self.make_file(normal, "qlog.zst")
    boot = self.make_file(self.make_segment("boot", bench=False), "bootlog.zst")
    self.assertEqual(self.uploader.next_file_to_upload(False)[2], str(boot))
    self.complete(boot)
    self.assertEqual(self.uploader.next_file_to_upload(False)[2], str(qlog))

  def test_all_bench_http_rejections_remain_pending_then_retry(self):
    segment = self.make_segment()
    recording = self.make_file(segment, "fcamera.hevc")
    key = f"{segment.name}/{recording.name}"
    for status in (401, 403, 412, 500):
      with self.subTest(status=status), patch("requests.put", return_value=SimpleNamespace(status_code=status)):
        self.assertFalse(self.uploader.step(1, False))
        self.assertIsNone(getxattr(str(recording), self.module["UPLOAD_ATTR_NAME"]))
        self.assertEqual(self.uploader.last_filename, "")
        self.assertEqual(self.uploader.next_file_to_upload(False)[1], key)
    response = SimpleNamespace(status_code=201, request=SimpleNamespace(headers={"Content-Length": "13"}))
    with patch("requests.put", return_value=response) as put:
      self.assertTrue(self.uploader.step(1, False))
      put.assert_called_once()
    self.assertEqual(getxattr(str(recording), self.module["UPLOAD_ATTR_NAME"]), b"1")
    self.assertIsNone(self.uploader.next_file_to_upload(False))

  def test_api_refusal_and_network_failure_are_not_success(self):
    segment = self.make_segment()
    recording = self.make_file(segment, "rlog.zst")
    self.api.get.return_value = SimpleNamespace(status_code=412)
    with patch("requests.put") as put:
      self.assertFalse(self.uploader.step(1, False))
      put.assert_not_called()
    self.api.get.return_value = SimpleNamespace(status_code=200, text='{"url": "https://upload.invalid/file", "headers": {}}')
    with patch("requests.put", side_effect=OSError("offline")):
      self.assertFalse(self.uploader.step(1, False))
    self.assertIsNone(getxattr(str(recording), self.module["UPLOAD_ATTR_NAME"]))

  def test_fakeupload_cannot_mark_bench_recordings_uploaded(self):
    segment = self.make_segment()
    recording = self.make_file(segment, "qlog.zst")
    self.globals["fake_upload"] = True
    self.assertFalse(self.uploader.step(1, False))
    self.assertIsNone(getxattr(str(recording), self.module["UPLOAD_ATTR_NAME"]))
    self.api.get.assert_not_called()

  def test_raw_logs_compressed_with_original_cloud_paths(self):
    for name in ("rlog", "qlog"):
      with self.subTest(name=name):
        segment = self.make_segment(f"00000003--0123456789--{name}")
        recording = self.make_file(segment, name)
        response = SimpleNamespace(status_code=200, request=SimpleNamespace(headers={"Content-Length": "13"}))
        with patch("requests.put", return_value=response):
          self.assertTrue(self.uploader.step(1, False))
        self.assertEqual(self.api.get.call_args.kwargs["path"], f"{segment.name}/{name}.zst")
        self.stream.assert_called_with(str(recording), True)

  def test_normal_route_http_policy_unchanged(self):
    segment = self.make_segment(bench=False)
    recording = self.make_file(segment, "qlog.zst")
    self.api.get.return_value = SimpleNamespace(status_code=412)
    self.assertTrue(self.uploader.step(1, False))
    self.assertEqual(getxattr(str(recording), self.module["UPLOAD_ATTR_NAME"]), b"1")

  def test_large_bench_qlog_not_silently_size_skipped(self):
    segment = self.make_segment()
    recording = self.make_file(segment, "qlog")
    with patch("os.path.getsize", return_value=26_000_000), patch("requests.put", side_effect=OSError("offline")) as put:
      self.assertFalse(self.uploader.step(1, False))
      put.assert_called_once()
    self.assertIsNone(getxattr(str(recording), self.module["UPLOAD_ATTR_NAME"]))


if __name__ == "__main__":
  unittest.main()
