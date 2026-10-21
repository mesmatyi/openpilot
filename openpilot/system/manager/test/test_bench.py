import os
import runpy
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

from openpilot.system.manager.bench import (
  ADAPTER_TIMEOUT, BENCH_ALWAYS, BENCH_KEYS, BENCH_ONROAD, LEASE_TIMEOUT,
  UNSAFE_ENV, BenchController, bench_ignition, fresh, hardware_safe, process_allowed, request_start,
)


class MockParams:
  def __init__(self):
    self.values = {}

  def get(self, key, **kwargs):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.get(key))

  def put(self, key, value, block=False):
    self.values[key] = value

  put_bool = put

  def remove(self, key):
    self.values.pop(key, None)


class MockHardware:
  def __init__(self):
    self.ps = SimpleNamespace(
      pandaType="tres", harnessStatus="notConnected", ignitionLine=False, ignitionCan=False,
      controlsAllowed=False, safetyModel="noOutput",
      **{f"canState{i}": SimpleNamespace(totalRxCnt=0) for i in range(3)},
    )
    self.values = {"pandaStates": [self.ps], "deviceState": SimpleNamespace(started=False)}
    self.valid = {"pandaStates": True, "deviceState": True}
    self.recv_time = {"pandaStates": 100.0}

  def __getitem__(self, key):
    return self.values[key]

  def all_checks(self, services):
    return all(self.valid[s] for s in services)


class TestBench(unittest.TestCase):
  def setUp(self):
    self.params = MockParams()
    self.sm = MockHardware()
    self.controller = BenchController(self.params, supported=True)
    self.env = patch.dict(os.environ, {}, clear=True)
    self.env.start()
    self.addCleanup(self.env.stop)

  def start(self):
    self.assertTrue(request_start(self.params, offroad=True, confirmed=True))
    self.assertTrue(self.controller.update(self.sm, False, now=100.0))

  def ready(self):
    self.start()
    self.params.put("BenchAdapterHeartbeat", 100.0)
    self.assertTrue(bench_ignition(self.params, self.sm, 100.0))
    self.sm["deviceState"].started = True
    self.controller.update(self.sm, False, now=100.1)

  def test_default_disabled_and_start_confirmation(self):
    self.assertFalse(self.controller.update(self.sm, False, now=100.0))
    self.assertFalse(process_allowed("benchd", False, self.params))
    for offroad, confirmed in ((False, False), (False, True), (True, False)):
      self.assertFalse(request_start(self.params, offroad, confirmed))
      self.assertFalse(self.params.get_bool("BenchModeRequested"))
    self.start()
    self.assertFalse(request_start(self.params, True, True))

  def test_start_requires_real_offroad_device(self):
    for field in ("unsupported", "onroad", "stale_device", "driver_view"):
      with self.subTest(field=field):
        self.setUp()
        if field == "unsupported":
          self.controller.supported = False
        elif field == "onroad":
          self.sm["deviceState"].started = True
        elif field == "stale_device":
          self.sm.valid["deviceState"] = False
        else:
          self.params.put_bool("IsDriverViewEnabled", True)
        self.params.put_bool("BenchModeRequested", True)
        self.assertFalse(self.controller.update(self.sm, False, now=100.0))
        self.assertFalse(self.params.get_bool("BenchMode"))
        self.assertFalse(self.params.get_bool("BenchModeRequested"))

  def test_legacy_simulation_environments_refused(self):
    for key in UNSAFE_ENV:
      with self.subTest(key=key), patch.dict(os.environ, {key: ""}):
        self.params.put_bool("BenchModeRequested", True)
        self.assertFalse(self.controller.update(self.sm, False, now=100.0))
        self.assertFalse(self.params.get_bool("BenchMode"))

  def test_raw_hardware_rejection(self):
    cases = (
      ("pandaType", "unknown"), ("harnessStatus", "normal"), ("ignitionLine", True),
      ("ignitionCan", True), ("controlsAllowed", True), ("safetyModel", "elm327"),
    )
    for field, value in cases:
      with self.subTest(field=field):
        original = getattr(self.sm.ps, field)
        setattr(self.sm.ps, field, value)
        self.assertFalse(hardware_safe(self.sm, 100.0))
        self.params.put_bool("BenchModeRequested", True)
        self.assertFalse(self.controller.update(self.sm, False, now=100.0))
        setattr(self.sm.ps, field, original)
    for i in range(3):
      cs = getattr(self.sm.ps, f"canState{i}")
      cs.totalRxCnt = 1
      self.assertFalse(hardware_safe(self.sm, 100.0))
      cs.totalRxCnt = 0
    self.sm.values["pandaStates"] = []
    self.assertFalse(hardware_safe(self.sm, 100.0))
    self.sm.values["pandaStates"] = [self.sm.ps, self.sm.ps]
    self.assertFalse(hardware_safe(self.sm, 100.0))

  def test_missing_invalid_stale_health(self):
    self.sm.valid["pandaStates"] = False
    self.assertFalse(hardware_safe(self.sm, 100.0))
    self.sm.valid["pandaStates"] = True
    self.assertFalse(hardware_safe(self.sm, 100.0 + ADAPTER_TIMEOUT))
    self.assertFalse(hardware_safe(self.sm, 99.0))
    for timestamp in (None, float("nan"), float("inf"), 101.0, 97.0):
      self.assertFalse(fresh(timestamp, 100.0, LEASE_TIMEOUT))

  def test_both_leases_required_for_synthetic_ignition(self):
    self.start()
    self.assertFalse(bench_ignition(self.params, self.sm, 100.0))
    self.params.put("BenchAdapterHeartbeat", 100.0)
    self.assertTrue(bench_ignition(self.params, self.sm, 100.0))
    self.params.put("BenchModeLease", 95.0)
    self.assertFalse(bench_ignition(self.params, self.sm, 100.0))
    self.params.put("BenchModeLease", 100.0)
    self.params.put("BenchAdapterHeartbeat", 98.0)
    self.assertFalse(bench_ignition(self.params, self.sm, 100.0))

  def test_stop_remains_available_onroad_and_recovers(self):
    self.ready()
    self.params.put_bool("BenchModeRequested", False)
    self.controller.update(self.sm, False, now=100.2)
    self.assertTrue(self.controller.stopping)
    self.assertFalse(bench_ignition(self.params, self.sm, 100.2))
    self.assertTrue(self.params.get_bool("BenchMode"))
    self.assertFalse(self.controller.finish_stop(self.sm, 100.2))
    self.sm["deviceState"].started = False
    self.assertTrue(self.controller.finish_stop(self.sm, 100.2))
    self.assertFalse(self.params.get_bool("BenchMode"))
    self.assertTrue(process_allowed("card", True, self.params))
    self.assertFalse(process_allowed("benchd", True, self.params))
    self.assertIsNone(self.params.get("CarParams"))
    self.assertFalse(self.params.get_bool("IsEngaged"))

  def test_connection_during_session_latches_until_disconnected(self):
    self.ready()
    self.sm.ps.ignitionLine = True
    self.controller.update(self.sm, False, now=100.2)
    self.sm["deviceState"].started = False
    self.assertFalse(self.controller.finish_stop(self.sm, 100.2))
    self.assertFalse(process_allowed("card", True, self.params))
    self.assertFalse(bench_ignition(self.params, self.sm, 100.2))
    self.sm.ps.ignitionLine = False
    self.assertTrue(self.controller.finish_stop(self.sm, 100.2))
    self.assertFalse(self.params.get_bool("BenchModeRequested"))

  def test_adapter_failure_and_staleness_fail_closed(self):
    self.ready()
    self.controller.update(self.sm, True, now=100.2)
    self.assertTrue(self.controller.stopping)
    self.assertIsNone(self.params.get("BenchModeLease"))
    self.assertFalse(process_allowed("benchd", True, self.params))

  def test_hung_adapter_after_start(self):
    self.ready()
    self.sm.recv_time["pandaStates"] = 101.0
    self.controller.update(self.sm, False, now=101.1)
    self.assertTrue(self.controller.stopping)
    self.assertFalse(bench_ignition(self.params, self.sm, 101.1))

  def test_adapter_start_timeout(self):
    self.start()
    self.sm.recv_time["pandaStates"] = 116.0
    self.controller.update(self.sm, False, now=116.0)
    self.assertTrue(self.controller.stopping)

  def test_hardware_startup_blockers_and_thermal_exit(self):
    self.start()
    self.params.put("BenchAdapterHeartbeat", 100.0)
    # hardwared remains the only owner of deviceState.started; a denied start stays offroad.
    self.controller.update(self.sm, False, now=100.1)
    self.assertFalse(self.sm["deviceState"].started)
    self.assertFalse(process_allowed("modeld", False, self.params))
    self.sm["deviceState"].started = True
    self.controller.update(self.sm, False, now=100.2)
    self.sm["deviceState"].started = False
    self.controller.update(self.sm, False, now=100.3)
    self.assertTrue(self.controller.stopping)
    self.assertIsNone(self.params.get("BenchModeLease"))

  def test_user_settings_and_real_car_caches_not_modified(self):
    protected = ("CalibrationParams", "CarParamsPersistent", "CarParamsCache", "CarParamsPrevRoute",
                 "LiveTorqueParameters", "LiveParametersV2", "LiveDelay", "LocationFilterInitialState",
                 "RecordFront", "RecordFrontLock", "DisableLogging", "GsmMetered", "OpenpilotEnabledToggle")
    for key in protected:
      self.params.put(key, f"original-{key}")
    originals = self.params.values.copy()
    self.ready()
    self.params.put("CarParams", b"synthetic")
    self.params.put_bool("BenchModeRequested", False)
    self.controller.update(self.sm, False, now=100.2)
    self.sm["deviceState"].started = False
    self.controller.finish_stop(self.sm, 100.2)
    for key, value in originals.items():
      self.assertEqual(self.params.get(key), value)


class TestBenchProcessSelection(unittest.TestCase):
  def load_config(self, params):
    class Process:
      def __init__(self, name, *args, enabled=True, **kwargs):
        self.name, self.enabled = name, enabled
        self.should_run = args[-1] if callable(args[-1]) else lambda *a: True

    def module(name, **attrs):
      mod = ModuleType(name)
      mod.__dict__.update(attrs)
      return mod

    mocks = {
      "opendbc.car.structs": module("opendbc.car.structs", car=SimpleNamespace(CarParams=object)),
      "openpilot.common.params": module("openpilot.common.params", Params=MockParams),
      "openpilot.common.hardware": module("openpilot.common.hardware", PC=False, COMMA_HARDWARE=True),
      "openpilot.system.manager.process": module("openpilot.system.manager.process", PythonProcess=Process, NativeProcess=Process, DaemonProcess=Process),
    }
    with patch.dict("sys.modules", mocks), patch.dict(os.environ, {}, clear=True):
      return runpy.run_path(str(Path(__file__).parents[1] / "process_config.py"))

  def test_actual_config_exclusive_publishers_and_privacy(self):
    params = MockParams()
    cfg = self.load_config(params)
    CP = SimpleNamespace(notCar=True)
    params.put_bool("BenchMode", True)
    params.put_bool("BenchModeRequested", True)
    for key in ("JoystickDebugMode", "LongitudinalManeuverMode", "LateralManeuverMode", "IsLiveStreaming"):
      params.put_bool(key, True)
    selected = {p.name for p in cfg["procs"] if p.enabled and p.should_run(True, params, CP)}
    gps = {"qcomgpsd"} if not cfg["ublox_available"]() else {"ubloxd", "pigeond"}
    expected = (BENCH_ALWAYS | BENCH_ONROAD | {"benchd"}) - {"qcomgpsd", "ubloxd", "pigeond"} | gps
    self.assertEqual(selected, expected)
    self.assertNotIn("card", selected)  # sole carState/carParams publisher is benchd
    self.assertIn("selfdrived", selected)  # sole selfdriveState publisher, explicitly interlocked
    self.assertNotIn("webcamerad", selected)
    for process in ("controlsd", "joystickd", "joystick", "radard", "plannerd", "paramsd", "torqued", "lagd",
                    "maneuversd", "lateral_maneuversd", "bridge", "webrtcd"):
      self.assertNotIn(process, selected)

    # A stale real-car CP must not override the bench logging privacy choice.
    params.put_bool("DisableLogging", True)
    self.assertFalse(cfg["logging"](True, params, SimpleNamespace(notCar=False)))
    self.assertFalse(cfg["logging"](True, params, CP))
    self.assertTrue(cfg["managed_processes"]["uploader"].should_run(True, params, CP))
    params.put_bool("BenchMode", False)
    self.assertTrue(cfg["managed_processes"]["card"].should_run(True, params, CP))
    self.assertFalse(cfg["managed_processes"]["benchd"].should_run(True, params, CP))

  def test_session_keys_are_reset_on_manager_start(self):
    keys = (Path(__file__).parents[3] / "common" / "params_keys.h").read_text()
    for key in BENCH_KEYS:
      line = next(line for line in keys.splitlines() if f'{{"{key}",' in line)
      self.assertIn("CLEAR_ON_MANAGER_START", line)
      self.assertNotIn("PERSISTENT", line)


class TestBenchAdapter(unittest.TestCase):
  def run_adapter(self, fault=None, warmup=False):
    params, sm = MockParams(), MockHardware()
    for key in ("BenchMode", "BenchModeRequested"):
      params.put_bool(key, True)
    params.put("BenchModeLease", 100.0)
    params.put = Mock(wraps=params.put)
    sm.seen = {"pandaStates": True}
    sm.valid["pandaStates"] = not warmup
    sm.update = lambda timeout: None
    pm = Mock()

    class Ratekeeper:
      frame = 0

      def __init__(self, rate):
        self.rate = rate

      def keep_time(self):
        self.frame += 1
        if warmup and self.frame == 2:
          sm.valid["pandaStates"] = True
        if self.frame == 3 and fault is not None:
          if fault == "lease":
            params.remove("BenchModeLease")
          else:
            sm.ps.ignitionLine = True
        if self.frame >= 12:
          params.put_bool("BenchModeRequested", False)

    def new_message(service, valid):
      content = SimpleNamespace(vEgo=0.0, cruiseState=SimpleNamespace(enabled=False, available=False))
      return SimpleNamespace(**{service: content}, valid=valid)

    cp = SimpleNamespace(to_bytes=lambda: b"bench-cp")
    car = SimpleNamespace(CarParams=SimpleNamespace(
      new_message=lambda: cp, SafetyModel=SimpleNamespace(noOutput="noOutput"),
      SafetyConfig=SimpleNamespace(new_message=lambda **kw: SimpleNamespace(**kw)),
    ))
    messaging = SimpleNamespace(PubMaster=Mock(return_value=pm), SubMaster=lambda services: sm, new_message=new_message)
    modules = {
      "opendbc.car.structs": SimpleNamespace(car=car),
      "openpilot.cereal": SimpleNamespace(messaging=messaging),
      "openpilot.common.params": SimpleNamespace(Params=lambda: params),
      "openpilot.common.realtime": SimpleNamespace(Ratekeeper=Ratekeeper),
    }
    with patch.dict("sys.modules", modules), patch("time.monotonic", return_value=100.0):
      adapter = runpy.run_path(str(Path(__file__).parents[1] / "benchd.py"))
      adapter["main"]()
    return cp, params, pm, messaging

  def test_stationary_passive_identity_and_single_ownership(self):
    cp, params, pm, messaging = self.run_adapter()
    self.assertTrue(cp.notCar)
    self.assertTrue(cp.passive)
    self.assertIn("BENCH", cp.carFingerprint)
    self.assertEqual(cp.safetyConfigs[0].safetyModel, "noOutput")
    messaging.PubMaster.assert_called_once_with(["carState", "carParams"])
    states = [call.args[1].carState for call in pm.send.call_args_list if call.args[0] == "carState"]
    self.assertEqual(len(states), 12)
    for state in states:
      self.assertTrue(state.standstill)
      self.assertEqual(state.vEgo, 0)
      self.assertFalse(state.canValid)
      self.assertFalse(state.cruiseState.enabled)
      self.assertFalse(state.cruiseState.available)
    self.assertEqual(params.get("CarParams"), b"bench-cp")
    self.assertIsNone(params.get("BenchAdapterHeartbeat"))
    self.assertEqual(sum(call.args[0] == "CarParams" for call in params.put.call_args_list), 1)

  def test_adapter_stops_publishing_on_connection_or_expired_manager(self):
    for fault in ("connection", "lease"):
      with self.subTest(fault=fault):
        _, params, pm, _ = self.run_adapter(fault)
        self.assertEqual(sum(call.args[0] == "carState" for call in pm.send.call_args_list), 3)
        self.assertIsNone(params.get("BenchAdapterHeartbeat"))

  def test_initial_seen_message_is_not_yet_frequency_checked(self):
    _, params, pm, _ = self.run_adapter(warmup=True)
    self.assertEqual(sum(call.args[0] == "carState" for call in pm.send.call_args_list), 10)
    self.assertEqual(params.get("CarParams"), b"bench-cp")
    self.assertIsNone(params.get("BenchAdapterHeartbeat"))


class TestBenchUI(unittest.TestCase):
  def test_confirmation_and_global_stop_overlay(self):
    params = MockParams()
    ui = SimpleNamespace(params=params, is_offroad=Mock(return_value=True))
    button, dialog = Mock(), Mock()
    gui = SimpleNamespace(big_ui=lambda: False, width=536, push_widget=Mock(), add_nav_stack_tick=Mock(), mouse_events=[])
    modules = {
      "pyray": SimpleNamespace(Rectangle=lambda *a: a),
      "openpilot.common.params": SimpleNamespace(Params=lambda: params),
      "openpilot.selfdrive.ui.ui_state": SimpleNamespace(ui_state=ui),
      "openpilot.system.ui.lib.application": SimpleNamespace(gui_app=gui),
      "openpilot.system.ui.widgets": SimpleNamespace(DialogResult=SimpleNamespace(CONFIRM=1, CANCEL=0)),
      "openpilot.system.ui.widgets.button": SimpleNamespace(Button=Mock(return_value=button), ButtonStyle=SimpleNamespace(DANGER=1)),
      "openpilot.system.ui.widgets.confirm_dialog": SimpleNamespace(ConfirmDialog=dialog),
    }
    with patch.dict("sys.modules", modules):
      widgets = runpy.run_path(str(Path(__file__).parents[3] / "selfdrive/ui/widgets/bench.py"))
      overlay = widgets["BenchOverlay"]()
      overlay.render()
      button.render.assert_not_called()
      widgets["confirm_bench_start"]()
      self.assertTrue(dialog.call_args.kwargs["rich"])
      callback = dialog.call_args.kwargs["callback"]
      callback(0)
      self.assertFalse(params.get_bool("BenchModeRequested"))
      callback(1)
      self.assertTrue(params.get_bool("BenchModeRequested"))
      params.put_bool("BenchMode", True)
      ui.is_offroad.return_value = False
      overlay.render()  # independent of settings/navigation stack and offroad state
      button.render.assert_called_once()
      stop = modules["openpilot.system.ui.widgets.button"].Button.call_args.kwargs["click_callback"]
      stop()
      self.assertFalse(params.get_bool("BenchModeRequested"))
      self.assertTrue(params.get_bool("BenchMode"))  # manager, not UI, releases the safety lock

  def test_banner_gestures_do_not_reach_covered_widgets(self):
    params = MockParams()
    params.put_bool("BenchMode", True)
    params.put_bool("BenchModeRequested", True)
    gui = SimpleNamespace(big_ui=lambda: False, width=536, mouse_events=[], show_touches=False, add_nav_stack_tick=Mock())
    def rectangle(x, y, width, height):
      return SimpleNamespace(x=x, y=y, width=width, height=height)

    ray = SimpleNamespace(Rectangle=rectangle, get_time=lambda: 100.0,
                          check_collision_point_rec=lambda pos, rect: rect.x <= pos.x < rect.x + rect.width and rect.y <= pos.y < rect.y + rect.height)
    modules = {
      "pyray": ray,
      "openpilot.common.params": SimpleNamespace(Params=lambda: params),
      "openpilot.selfdrive.ui.ui_state": SimpleNamespace(device=SimpleNamespace(awake=True)),
      "openpilot.system.ui.lib.application": SimpleNamespace(
        gui_app=gui, MousePos=object, MouseEvent=object, MAX_TOUCH_SLOTS=2),
    }
    with patch.dict("sys.modules", modules):
      base = runpy.run_path(str(Path(__file__).parents[3] / "system/ui/widgets/__init__.py"))

      class ClickWidget(base["Widget"]):
        def _render(self, rect):
          pass

      def button(text, click_callback, **kwargs):
        widget = ClickWidget()
        widget.set_click_callback(click_callback)
        return widget

      with patch.dict("sys.modules", {
        "openpilot.system.ui.widgets": SimpleNamespace(DialogResult=base["DialogResult"]),
        "openpilot.system.ui.widgets.button": SimpleNamespace(Button=button, ButtonStyle=SimpleNamespace(DANGER=1)),
        "openpilot.system.ui.widgets.confirm_dialog": SimpleNamespace(ConfirmDialog=Mock()),
        "openpilot.selfdrive.ui.ui_state": SimpleNamespace(ui_state=Mock()),
      }):
        overlay = runpy.run_path(str(Path(__file__).parents[3] / "selfdrive/ui/widgets/bench.py"))["BenchOverlay"]()
        covered = ClickWidget()
        changed_privacy = Mock()
        covered.set_click_callback(changed_privacy)
        for y, pressed, released in ((20, True, False), (180, False, False), (20, False, False), (20, False, True)):
          gui.mouse_events[:] = [SimpleNamespace(
            pos=SimpleNamespace(x=200, y=y), slot=0, left_pressed=pressed, left_released=released, t=100.0)]
          overlay.capture_input()
          self.assertEqual(gui.mouse_events, [])
          covered.render(rectangle(0, 0, 536, 240))
          overlay.render()
        changed_privacy.assert_not_called()
        self.assertFalse(params.get_bool("BenchModeRequested"))
        self.assertTrue(params.get_bool("BenchMode"))
        # Non-banner gestures must remain usable.
        for pressed, released in ((True, False), (False, True)):
          gui.mouse_events[:] = [SimpleNamespace(
            pos=SimpleNamespace(x=200, y=180), slot=0, left_pressed=pressed, left_released=released, t=100.0)]
          overlay.capture_input()
          covered.render(rectangle(0, 0, 536, 240))
          overlay.render()
        changed_privacy.assert_called_once()

  def test_capability_refresh_preserves_persistent_driving_preferences(self):
    params = MockParams()
    params.put_bool("BenchMode", True)
    params.put_bool("AlphaLongitudinalEnabled", True)
    params.put_bool("ExperimentalMode", True)
    ui = SimpleNamespace(params=params, CP=SimpleNamespace(alphaLongitudinalAvailable=False),
                         update_params=Mock(), has_longitudinal_control=False, is_offroad=lambda: True,
                         is_release=False, engaged=False)
    modules = {name: MagicMock() for name in (
      "openpilot.cereal", "openpilot.common.time_helpers", "openpilot.selfdrive.ui.widgets.ssh_key",
      "openpilot.system.ui.widgets.list_view", "openpilot.system.ui.widgets.confirm_dialog",
      "openpilot.selfdrive.ui.widgets.bench", "openpilot.system.ui.lib.application",
      "openpilot.selfdrive.ui.mici.widgets.button", "openpilot.selfdrive.ui.layouts.settings.common",
    )}
    modules.update({
      "openpilot.common.params": SimpleNamespace(Params=lambda: params, UnknownKeyName=KeyError),
      "openpilot.system.ui.widgets": SimpleNamespace(Widget=object, DialogResult=Mock()),
      "openpilot.system.ui.widgets.scroller": SimpleNamespace(NavScroller=object),
      "openpilot.system.ui.widgets.scroller_tici": SimpleNamespace(Scroller=object),
      "openpilot.system.ui.lib.multilang": SimpleNamespace(tr=lambda s: s, tr_noop=lambda s: s),
      "openpilot.selfdrive.ui.ui_state": SimpleNamespace(ui_state=ui),
      "openpilot.selfdrive.ui.mici.widgets.dialog": SimpleNamespace(
        BigDialog=Mock(), BigInputDialog=Mock(), BigConfirmationCircleButton=Mock(), SettingDescriptionDialog=object),
    })
    with patch.dict("sys.modules", modules):
      for subdir, class_name in (("layouts", "DeveloperLayout"), ("mici/layouts", "DeveloperLayoutMici"),
                                 ("layouts", "TogglesLayout"), ("mici/layouts", "TogglesLayoutMici")):
        filename = "developer" if "Developer" in class_name else "toggles"
        source = runpy.run_path(str(Path(__file__).parents[3] / f"selfdrive/ui/{subdir}/settings/{filename}.py"))
        layout = object.__new__(source[class_name])
        layout.__dict__.update({name: Mock() for name in (
          "_joystick_toggle", "_long_maneuver_toggle", "_lat_maneuver_toggle", "_alpha_long_toggle",
          "_adb_toggle", "_ssh_toggle", "_ui_debug_toggle", "_experimental_btn", "_personality_toggle",
          "_long_personality_setting", "_update_experimental_mode_icon",
        )})
        layout._params, layout._is_release = params, False
        layout._refresh_toggles, layout._toggle_defs = (), {}
        layout._toggles = {"ExperimentalMode": Mock()}
        layout._update_toggles()
        self.assertTrue(params.get_bool("AlphaLongitudinalEnabled"), class_name)
        self.assertTrue(params.get_bool("ExperimentalMode"), class_name)

  def test_road_view_and_awake_without_spoofing_raw_ignition(self):
    params = MockParams()
    params.put_bool("BenchMode", True)
    sm = MockHardware()
    sm.values.update({"selfdriveState": SimpleNamespace(enabled=True)})
    sm.updated = dict.fromkeys(("pandaStates", "wideRoadCameraState"), False)
    sm.alive = {"pandaStates": True, "wideRoadCameraState": False}
    modules = {name: MagicMock() for name in (
      "numpy", "openpilot.cereal", "opendbc.car.structs", "openpilot.common.filter_simple",
      "openpilot.common.swaglog", "openpilot.common.realtime", "openpilot.common.hardware.usb",
      "openpilot.selfdrive.ui.lib.prime_state", "openpilot.selfdrive.modeld.helpers",
    )}
    gui = SimpleNamespace(target_fps=60, mouse_events=[], big_ui=lambda: False, set_should_render=Mock())
    hardware = Mock()
    hardware.get_device_type.return_value = "mici"
    modules.update({
      "openpilot.common.params": SimpleNamespace(Params=lambda: params),
      "openpilot.common.hardware": SimpleNamespace(HARDWARE=hardware, PC=False),
      "openpilot.system.ui.lib.application": SimpleNamespace(gui_app=gui),
    })
    with patch.dict("sys.modules", modules):
      state = runpy.run_path(str(Path(__file__).parents[3] / "selfdrive/ui/ui_state.py"))
      ui, device = state["ui_state"], state["device"]
      ui.sm, ui.CP = sm, SimpleNamespace(notCar=True)
      ui.ignition = False
      sm["deviceState"].started = True
      ui._update_state()
      self.assertTrue(ui.started)
      self.assertFalse(ui.ignition)
      self.assertFalse(ui.is_body)
      self.assertFalse(ui.engaged)
      device._awake = False
      device._update_wakefulness()
      self.assertTrue(device.awake)
      params.put_bool("BenchMode", False)
      ui._update_state()
      self.assertFalse(ui.started)
      self.assertTrue(ui.is_body)  # normal notCar/body behavior is restored


class TestBenchEngagement(unittest.TestCase):
  def test_selfdrived_interlock_overrides_enable_events_and_nonpassive_cp(self):
    modules = {name: MagicMock() for name in (
      "openpilot.cereal", "openpilot.cereal.messaging", "opendbc.car.structs", "openpilot.cereal.visionipc", "msgq.visionipc",
      "openpilot.common.params", "openpilot.common.realtime", "openpilot.common.swaglog", "openpilot.common.gps",
      "openpilot.selfdrive.car.car_events", "openpilot.selfdrive.locationd.helpers",
      "openpilot.selfdrive.selfdrived.events", "openpilot.selfdrive.selfdrived.helpers",
      "openpilot.selfdrive.selfdrived.state", "openpilot.selfdrive.selfdrived.alertmanager",
      "openpilot.common.version", "openpilot.common.hardware",
    )}
    with patch.dict("sys.modules", modules):
      source = runpy.run_path(str(Path(__file__).parents[3] / "selfdrive/selfdrived/selfdrived.py"))
      sd = object.__new__(source["SelfdriveD"])
      sd.bench_mode = True  # latched at process initialization, independent of later request/CP changes
      sd.CP = SimpleNamespace(passive=False)
      sd.initialized = sd.enabled = sd.active = True
      sd.state_machine = Mock(state=source["State"].enabled)
      sd.events = Mock()
      sd.state_machine.update.return_value = (True, True)
      sd.data_sample = Mock(return_value=SimpleNamespace(cruiseState=SimpleNamespace(enabled=True)))
      sd.update_events, sd.update_alerts, sd.publish_selfdriveState = Mock(), Mock(), Mock()
      sd.sm = SimpleNamespace(frame=1)
      sd.step()
      self.assertFalse(sd.enabled)
      self.assertFalse(sd.active)
      self.assertEqual(sd.state_machine.state, source["State"].disabled)
      sd.state_machine.update.assert_not_called()
      sd.publish_selfdriveState.assert_called_once()
      sd.bench_mode = False
      sd.step()
      sd.state_machine.update.assert_called_once()  # normal mode is unchanged
      params = MockParams()
      for key in ("BenchMode", "AlphaLongitudinalEnabled", "ExperimentalMode"):
        params.put_bool(key, True)
      modules["openpilot.common.params"].Params.return_value = params
      cp = SimpleNamespace(brand="mock", alphaLongitudinalAvailable=False, openpilotLongitudinalControl=False)
      initialized = source["SelfdriveD"](cp)
      self.assertTrue(params.get_bool("AlphaLongitudinalEnabled"))
      self.assertTrue(params.get_bool("ExperimentalMode"))
      initialized.publish_selfdriveState(Mock())
      packet = source["messaging"].new_message.return_value.selfdriveState
      self.assertFalse(packet.enabled)
      self.assertFalse(packet.active)
      self.assertFalse(packet.engageable)
      params.put_bool("BenchMode", False)
      source["SelfdriveD"](cp)
      self.assertIsNone(params.get("AlphaLongitudinalEnabled"))
      self.assertIsNone(params.get("ExperimentalMode"))  # ordinary capability cleanup still applies


if __name__ == "__main__":
  unittest.main()
