import math
import numpy as np

from cereal import log
from opendbc.car.interfaces import LatControlInputs
from opendbc.car.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

class LatControlLQR(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.mpc_frame = 0

    self.scale = 1700.0
    self.ki = 0.01

    self.A = np.array([0.,1.,-0.2261, 1.2182]).reshape((2, 2))
    self.B = np.array([-1.92,3.95]).reshape((2, 1))
    self.C = np.array([1.,0.]).reshape((1, 2))
    self.K = np.array([-110.73,451.22]).reshape((1, 2))
    self.L = np.array([0.32,0.31]).reshape((2, 1))
    self.dc_gain = 0.0027

    self.x_hat = np.array([[0], [0]])
    self.i_unwind_rate = 0.3
    self.i_rate = 1.0

    self.reset()

    self.ll_timer = 0

  def reset(self):
    super().reset()
    self.i_lqr = 0.0

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    pass


  def update(self, active, CS, VM, params, steer_limited_by_controls, desired_curvature, calibrated_pose, curvature_limited):
    self.ll_timer += 1
    if self.ll_timer > 100:
      self.ll_timer = 0

    lqr_log = log.ControlsState.LateralTorqueState.new_message()

    torque_scale = (0.45 + CS.vEgo / 60.0)**2  # Scale actuator model with speed

    # Subtract offset. Zero angle should correspond to zero torque
    steering_angle_no_offset = CS.steeringAngleDeg - params.angleOffsetAverageDeg

    desired_angle = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))

    instant_offset = params.angleOffsetDeg - params.angleOffsetAverageDeg
    desired_angle += instant_offset  # Only add offset that originates from vehicle model errors

    # Update Kalman filter
    angle_steers_k = float(self.C.dot(self.x_hat))
    e = steering_angle_no_offset - angle_steers_k
    self.x_hat = self.A.dot(self.x_hat) + self.B.dot(CS.steeringTorqueEps / torque_scale) + self.L.dot(e)

    if not active:
      lqr_output = 0.
      output_steer = 0.
      self.reset()
    else:

      # LQR
      u_lqr = float(desired_angle / self.dc_gain - self.K.dot(self.x_hat))
      lqr_output = torque_scale * u_lqr / self.scale

      # Integrator
      if CS.steeringPressed:
        self.i_lqr -= self.i_unwind_rate * float(np.sign(self.i_lqr))
      else:
        error = desired_angle - angle_steers_k
        i = self.i_lqr + self.ki * self.i_rate * error
        control = lqr_output + i

        if (error >= 0 and (control <= self.steer_max or i < 0.0)) or \
           (error <= 0 and (control >= -self.steer_max or i > 0.0)):
          self.i_lqr = i

      output_steer = lqr_output + self.i_lqr
      output_steer = np.clip(output_steer, -self.steer_max, self.steer_max)

    return output_steer, desired_angle, lqr_log