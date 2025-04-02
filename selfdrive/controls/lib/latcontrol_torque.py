import math
import numpy as np

from cereal import log
from opendbc.car.interfaces import LatControlInputs
from opendbc.car.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]


class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg

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

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update(self, active, CS, VM, params, steer_limited_by_controls, desired_curvature, calibrated_pose, curvature_limited):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:

      #### LQR start

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

      # LQR
      u_lqr = float(desired_angle / self.dc_gain - self.K.dot(self.x_hat))
      lqr_output = torque_scale * u_lqr / self.scale


      error = desired_angle - angle_steers_k
      i = self.i_lqr + self.ki * self.i_rate * error
      control = lqr_output + i

      if (error >= 0 and (control <= self.steer_max or i < 0.0)) or \
          (error <= 0 and (control >= -self.steer_max or i > 0.0)):
        self.i_lqr = i

      output_steer = (lqr_output + self.i_lqr)
      output_steer = np.clip(output_steer, -self.steer_max, self.steer_max)


      #### LQR end #####


      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      if self.use_steering_angle:
        actual_curvature = actual_curvature_vm
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      else:
        assert calibrated_pose is not None
        actual_curvature_pose = calibrated_pose.angular_velocity.yaw / CS.vEgo
        actual_curvature = np.interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_pose])
        curvature_deadzone = 0.0
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      gravity_adjusted_lateral_accel = desired_lateral_accel - roll_compensation
      torque_from_setpoint = self.torque_from_lateral_accel(LatControlInputs(setpoint, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                                            setpoint, lateral_accel_deadzone, friction_compensation=False, gravity_adjusted=False)
      torque_from_measurement = self.torque_from_lateral_accel(LatControlInputs(measurement, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                                               measurement, lateral_accel_deadzone, friction_compensation=False, gravity_adjusted=False)
      pid_log.error = float(torque_from_setpoint - torque_from_measurement)
      ff = self.torque_from_lateral_accel(LatControlInputs(gravity_adjusted_lateral_accel, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                          desired_lateral_accel - actual_lateral_accel, lateral_accel_deadzone, friction_compensation=True,
                                          gravity_adjusted=True)

      freeze_integrator = steer_limited_by_controls or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)




      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(actual_lateral_accel)
      pid_log.desiredLateralAccel = float(desired_lateral_accel)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_controls, curvature_limited))
      print(-output_torque)

      output_torque = output_steer

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
