"""ROS 2 microphone node.

Topics
  audio/raw                 comma_msgs/AudioChunk  16 kHz mono S16LE, 50 ms chunks
  audio/sound_pressure_db   std_msgs/Float32       A-weighted level, 10 Hz (uncalibrated, like openpilot micd)
"""
import queue
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

from comma_msgs.msg import AudioChunk
from comma_audio.spl import SplMeter


class AudioNode(Node):
  def __init__(self):
    super().__init__("audio")
    self.sample_rate = self.declare_parameter("sample_rate", 16000).value
    self.chunk = self.declare_parameter("chunk_samples", 800).value
    self.device = self.declare_parameter("device", "").value  # PortAudio device name/index, "" = default
    self.frame_id = self.declare_parameter("frame_id", "microphone").value

    self.pub_raw = self.create_publisher(AudioChunk, "audio/raw", qos_profile_sensor_data)
    self.pub_spl = self.create_publisher(Float32, "audio/sound_pressure_db", qos_profile_sensor_data)
    self.meter = SplMeter(window=self.sample_rate // 10, sample_rate=self.sample_rate)
    self.q: queue.Queue = queue.Queue(maxsize=100)
    self.stream = None
    self.open_stream()
    self.create_timer(0.01, self.drain)

  def open_stream(self) -> None:
    import sounddevice as sd  # PortAudio; imported lazily so tests do not need it

    for attempt in range(10):
      try:
        self.stream = sd.InputStream(channels=1, samplerate=self.sample_rate, blocksize=self.chunk, dtype="float32",
                                     device=self.device or None, callback=self.callback)
        self.stream.start()
        self.get_logger().info(f"microphone stream started: {self.stream.device=} {self.sample_rate} Hz")
        return
      except Exception as e:
        self.get_logger().warning(f"could not open microphone ({e}), retrying ({attempt + 1}/10)")
        time.sleep(3)
    raise RuntimeError("no microphone available")

  def callback(self, indata, frames, _time, status):
    # PortAudio thread: hand off quickly; stamp with the capture time of the first sample
    t0 = time.time_ns() - int(frames * 1e9 / self.sample_rate)
    try:
      self.q.put_nowait((t0, np.frombuffer(indata, dtype=np.float32).copy()))
    except queue.Full:
      pass

  def drain(self) -> None:
    while True:
      try:
        t0, samples = self.q.get_nowait()
      except queue.Empty:
        return
      msg = AudioChunk(sample_rate=self.sample_rate, channels=1, encoding="S16LE")
      msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(t0, 1_000_000_000)
      msg.header.frame_id = self.frame_id
      msg.data = (np.clip(samples, -1.0, 1.0) * 32767).astype("<i2").tobytes()
      self.pub_raw.publish(msg)
      for _, _, db in self.meter.feed(samples.astype(np.float64)):
        self.pub_spl.publish(Float32(data=float(db)))

  def destroy_node(self):
    if self.stream is not None:
      self.stream.stop()
      self.stream.close()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = AudioNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
  main()
