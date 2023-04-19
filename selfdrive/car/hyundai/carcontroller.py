from random import randint

from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_driver_steer_torque_limits
from selfdrive.car.hyundai import hyundaicanfd, hyundaican, hyundaican_community
from selfdrive.car.hyundai.hyundaicanfd import CanBus
from selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR, \
  LEGACY_SAFETY_MODE_CAR, CAN_GEARS
from selfdrive.car.interfaces import ACCEL_MAX, ACCEL_MIN
from selfdrive.controls.neokii.cruise_state_manager import CruiseStateManager
from selfdrive.controls.neokii.navi_controller import SpeedLimiter
from selfdrive.controls.neokii.speed_controller import CREEP_SPEED

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.angle_limit_counter = 0
    self.frame = 0

    self.accel_last = 0
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0

    self.resume_cnt = 0
    self.last_lead_distance = 0
    self.resume_wait_timer = 0

    from common.params import Params
    params = Params()
    self.ldws_opt = params.get_bool('IsLdwsCar')
    self.e2e_long = params.get_bool('ExperimentalMode')

    self.stock_accel_weight = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    if not CC.latActive:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    # accel + longitudinal
    accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)
    left_lane_warning = right_lane_warning = SpeedLimiter.instance().get_cam_alert()
    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and self.CP.openpilotLongitudinalControl and self.CP.sccBus == 0:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append([addr, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus])

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append([0x7b1, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", self.CAN.ECAN])

    # >90 degree steering fault prevention
    # Count up to MAX_ANGLE_FRAMES, at which point we need to cut torque to avoid a steering fault
    if CC.latActive and abs(CS.out.steeringAngleDeg) >= MAX_ANGLE:
      self.angle_limit_counter += 1
    else:
      self.angle_limit_counter = 0

    # Cut steer actuation bit for two frames and hold torque with induced temporary fault
    torque_fault = CC.latActive and self.angle_limit_counter > MAX_ANGLE_FRAMES
    lat_active = CC.latActive and not torque_fault

    if self.angle_limit_counter >= MAX_ANGLE_FRAMES + MAX_ANGLE_CONSECUTIVE_FRAMES:
      self.angle_limit_counter = 0

    # CAN-FD platforms
    if self.CP.carFingerprint in CANFD_CAR:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      hda2_long = hda2 and self.CP.openpilotLongitudinalControl

      # steering control
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, lat_active, apply_steer))

      # disable LFA on HDA2
      if self.frame % 5 == 0 and hda2:
        can_sends.append(hyundaicanfd.create_cam_0x2a4(self.packer, self.CAN, CS.cam_0x2a4))

      # LFA and HDA icons
      if self.frame % 5 == 0 and (not hda2 or hda2_long):
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

      # blinkers
      if hda2 and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, CC.leftBlinker, CC.rightBlinker))

      if self.CP.openpilotLongitudinalControl:
        if hda2:
          can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                           set_speed_in_units))
          self.accel_last = accel
      else:
        # button presses
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
          # cruise cancel
          if CC.cruiseControl.cancel:
            if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
              can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CAN, CS.cruise_info))
              self.last_button_frame = self.frame
            else:
              for _ in range(20):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.CANCEL))
              self.last_button_frame = self.frame

          # cruise standstill resume
          elif CC.cruiseControl.resume:
            if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
              # TODO: resume for alt button cars
              pass
            else:
              for _ in range(20):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.RES_ACCEL))
              self.last_button_frame = self.frame
    else:
      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.car_fingerprint, apply_steer, lat_active,
                                                torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                left_lane_warning, right_lane_warning, self.ldws_opt, self.CP))

      if not self.CP.openpilotLongitudinalControl or CruiseStateManager.instance().is_resume_spam_allowed(self.CP):
        if CC.cruiseControl.cancel:
          can_sends.append(hyundaican.create_clu11(self.packer, CS.clu11, Buttons.CANCEL, self.CP.sccBus))
        elif CC.cruiseControl.resume:
          if self.CP.carFingerprint in LEGACY_SAFETY_MODE_CAR:
            self.update_auto_resume(CC, CS, can_sends)
          else:
            # send resume at a max freq of 10Hz
            if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
              # send 25 messages at a time to increases the likelihood of resume being accepted
              can_sends.extend([hyundaican.create_clu11(self.packer, CS.clu11, Buttons.RES_ACCEL, self.CP.sccBus)] * 25)
              if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
                self.last_button_frame = self.frame

      if self.CP.carFingerprint in CAN_GEARS["send_mdps12"]:  # send mdps12 to LKAS to prevent LKAS error
        can_sends.append(hyundaican_community.create_mdps12(self.packer, self.frame, CS.mdps12))

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
        # TODO: unclear if this is needed
        jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0

        if CC.longActive:
          start_boost = interp(CS.out.vEgo, [CREEP_SPEED, 1.6 * CREEP_SPEED], [0.2 if self.e2e_long else 0.5, 0.0])
          is_accelerating = interp(accel, [0.0, 0.2], [0.0, 1.0])
          boost = start_boost * is_accelerating
          accel += boost

        stock_cam = False
        if self.CP.sccBus == 2:
          aReqValue = CS.scc12["aReqValue"]
          if not SpeedLimiter.instance().get_active() and ACCEL_MIN <= aReqValue <= ACCEL_MAX:
            accel, stock_cam = self.get_stock_cam_accel(accel, aReqValue, CS.scc11)

        if self.CP.sccBus == 0:
          can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                        hud_control.leadVisible, set_speed_in_units, stopping,
                                                          CC.cruiseControl.override, CS, stock_cam))
        else:
          can_sends.extend(hyundaican_community.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                          hud_control.leadVisible, set_speed_in_units, stopping,
                                                          CC.cruiseControl.override, CS, stock_cam))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
        if self.CP.sccBus == 0:
          can_sends.extend(hyundaican.create_acc_opt(self.packer))
        elif CS.scc13 is not None:
          can_sends.append(hyundaican_community.create_acc_opt(self.packer, CS))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl and self.CP.sccBus == 0:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled, SpeedLimiter.instance().get_active()))

    CC.applyAccel = accel

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends

  def update_auto_resume(self, CC, CS, can_sends):
    # neokii
    if CS.lead_distance <= 0:
      return

    if CC.cruiseControl.resume and not CS.out.gasPressed:
      if self.last_lead_distance == 0:
        self.last_lead_distance = CS.lead_distance
        self.resume_cnt = 0
        self.resume_wait_timer = 0

      elif self.resume_wait_timer > 0:
        self.resume_wait_timer -= 1

      elif abs(CS.lead_distance - self.last_lead_distance) > 0.1:
        can_sends.append(hyundaican.create_clu11(self.packer, CS.clu11, Buttons.RES_ACCEL, self.CP.sccBus))
        self.resume_cnt += 1

        if self.resume_cnt >= int(randint(4, 5) * 2):
          self.resume_cnt = 0
          self.resume_wait_timer = int(randint(20, 25) * 2)

    elif self.last_lead_distance != 0:
      self.last_lead_distance = 0


  def get_stock_cam_accel(self, apply_accel, stock_accel, scc11):
    stock_cam = scc11["Navi_SCC_Camera_Act"] == 2 and scc11["Navi_SCC_Camera_Status"] == 2
    if stock_cam:
      self.stock_accel_weight += DT_CTRL / 3.
    else:
      self.stock_accel_weight -= DT_CTRL / 3.

    self.stock_weight = clip(self.stock_accel_weight, 0., 1.)

    accel = stock_accel * self.stock_weight + apply_accel * (1. - self.stock_weight)
    return min(accel, apply_accel), stock_cam

