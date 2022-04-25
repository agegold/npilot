import math
from common.numpy_fast import interp
from selfdrive.controls.lib.latcontrol_pid import ERROR_RATE_FRAME
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from cereal import log
from selfdrive.ntune import nTune

LOW_SPEED_FACTOR = 200
JERK_THRESHOLD = 0.2

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.pid = PIDController(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.ki,
                            k_f=CP.lateralTuning.torque.kf, pos_limit=1.0, neg_limit=-1.0)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    self.steer_max = 1.0
    self.pid.pos_limit = self.steer_max
    self.pid.neg_limit = -self.steer_max
    self.use_steering_angle = CP.lateralTuning.torque.useSteeringAngle
    self.friction = CP.lateralTuning.torque.friction
    self.tune = nTune(CP, self)

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, CP, VM, params, last_actuators, desired_curvature, desired_curvature_rate, llk):
    self.tune.check()
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      else:
        actual_curvature = llk.angularVelocityCalibrated.value[2] / CS.vEgo
      desired_lateral_accel = desired_curvature * CS.vEgo**2
      desired_lateral_jerk = desired_curvature_rate * CS.vEgo**2
      actual_lateral_accel = actual_curvature * CS.vEgo**2

      setpoint = desired_lateral_accel + LOW_SPEED_FACTOR * desired_curvature
      measurement = actual_lateral_accel + LOW_SPEED_FACTOR * actual_curvature
      error = setpoint - measurement

      deadzone = interp(CS.vEgo, CP.lateralTuning.torque.deadzoneBP, CP.lateralTuning.torque.deadzoneV)
      error_deadzone = apply_deadzone(error, deadzone)

      pid_log.error = error_deadzone

      ff = desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY
      output_torque = self.pid.update(error_deadzone,
                                      override=CS.steeringPressed, feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=CS.steeringRateLimited)

      friction_compensation = interp(desired_lateral_jerk, [-JERK_THRESHOLD, JERK_THRESHOLD], [-self.friction, self.friction])
      output_torque += friction_compensation

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS)

    angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll)) + params.angleOffsetDeg

    #TODO left is positive in this convention
    return -output_torque, angle_steers_des, pid_log
