import random

import numpy as np
from common.numpy_fast import clip, interp
from cereal import car
from common.conversions import Conversions as CV
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MIN, V_CRUISE_MAX, V_CRUISE_ENABLE_MIN, V_CRUISE_UNSET
from selfdrive.controls.neokii.cruise_state_manager import CruiseStateManager, V_CRUISE_DELTA_KM, V_CRUISE_DELTA_MI, \
  V_CRUISE_MIN_CRUISE_STATE
from selfdrive.car.hyundai.values import Buttons
from common.params import Params
from selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE
from selfdrive.controls.neokii.navi_controller import SpeedLimiter
from selfdrive.controls.ntune import ntune_common_get
from selfdrive.controls.radard import RADAR_TO_CAMERA

SYNC_MARGIN = 3.
CREEP_SPEED = 2.3


MIN_CURVE_SPEED = 32. * CV.KPH_TO_MS

EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type


class SpeedController:

  def __init__(self, CP, CI):

    self.CP = CP
    self.CI = CI

    self.long_control = CP.openpilotLongitudinalControl

    self.params = Params()
    self.read_param()

    self.target_speed = 0.

    self.wait_timer = 0
    self.alive_timer = 0
    self.btn = Buttons.NONE

    self.alive_index = 0
    self.wait_index = 0
    self.alive_count = 0

    self.wait_count_list, self.alive_count_list = CI.get_params_adjust_set_speed()
    random.shuffle(self.wait_count_list)
    random.shuffle(self.alive_count_list)

    self.slowing_down = False
    self.slowing_down_alert = False
    self.slowing_down_sound_alert = False
    self.active_cam = False

    self.max_speed_clu = 0.
    self.curve_speed_ms = 0.

    self.cruise_speed_kph = 0.
    self.real_set_speed_kph = 0.

    self.prev_cruise_enabled = False

    self.prev_button = ButtonType.unknown
    self.button_count = 0
    self.long_pressed = False

    self.limited_lead = False

  def kph_to_clu(self, kph):
    return int(kph * CV.KPH_TO_MS * self.speed_conv_to_clu)

  def get_alive_count(self):
    count = self.alive_count_list[self.alive_index]
    self.alive_index += 1
    if self.alive_index >= len(self.alive_count_list):
      self.alive_index = 0
    return count

  def get_wait_count(self):
    count = self.wait_count_list[self.wait_index]
    self.wait_index += 1
    if self.wait_index >= len(self.wait_count_list):
      self.wait_index = 0
    return count

  def read_param(self):
    self.slow_on_curves = True #self.params.get_bool('SlowOnCurves')
    self.sync_set_speed_while_gas_pressed = True #self.params.get_bool('SyncGasPressed')
    self.is_metric = self.params.get_bool('IsMetric')
    self.show_debug_message = self.params.get_bool('ShowDebugMessage')

    self.speed_conv_to_ms = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
    self.speed_conv_to_clu = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    self.min_set_speed_clu = self.kph_to_clu(V_CRUISE_MIN) # TODO - neokii
    self.max_set_speed_clu = self.kph_to_clu(V_CRUISE_MAX)

  def reset(self):

    self.wait_timer = 0
    self.alive_timer = 0
    self.btn = Buttons.NONE
    self.target_speed = 0.

    self.max_speed_clu = 0.
    self.curve_speed_ms = 0.

    self.slowing_down = False
    self.slowing_down_alert = False
    self.slowing_down_sound_alert = False

  def inject_events(self, CS, events):

    if CS.cruiseState.enabled:
      if self.slowing_down_sound_alert:
        self.slowing_down_sound_alert = False
        events.add(EventName.slowingDownSpeedSound)
      elif self.slowing_down_alert:
        events.add(EventName.slowingDownSpeed)

  def cal_max_speed(self, CS, sm, clu_speed, v_cruise_kph):

    # kph
    apply_limit_speed, road_limit_speed, left_dist, first_started, max_speed_log = \
      SpeedLimiter.instance().get_max_speed(clu_speed, self.is_metric)

    curv_limit = 0
    self.cal_curve_speed(sm, CS.vEgo, sm.frame)
    if self.slow_on_curves and self.curve_speed_ms >= MIN_CURVE_SPEED:
      max_speed_clu = min(v_cruise_kph * CV.KPH_TO_MS, self.curve_speed_ms) * self.speed_conv_to_clu
      curv_limit = int(max_speed_clu)
    else:
      max_speed_clu = self.kph_to_clu(v_cruise_kph)

    self.active_cam = road_limit_speed > 0 and left_dist > 0

    if apply_limit_speed >= self.kph_to_clu(10):

      if first_started:
        self.max_speed_clu = clu_speed

      max_speed_clu = min(max_speed_clu, apply_limit_speed)

      if clu_speed > apply_limit_speed:

        if not self.slowing_down_alert and not self.slowing_down:
          self.slowing_down_sound_alert = True
          self.slowing_down = True

        self.slowing_down_alert = True

      else:
        self.slowing_down_alert = False

    else:
      self.slowing_down_alert = False
      self.slowing_down = False

    lead_speed = self.get_long_lead_speed(clu_speed, sm)

    if lead_speed >= self.min_set_speed_clu:
      if lead_speed < max_speed_clu:
        max_speed_clu = lead_speed

        if not self.limited_lead:
          self.max_speed_clu = clu_speed + 3.
          self.limited_lead = True
    else:
      self.limited_lead = False

    self.update_max_speed(int(round(max_speed_clu)), curv_limit != 0 and curv_limit == int(max_speed_clu))
    return max_speed_clu

  def get_lead(self, sm):

    radar = sm['radarState']
    if radar.leadOne.status:
      return radar.leadOne

    return None

  def get_long_lead_speed(self, clu11_speed, sm):

    lead = self.get_lead(sm)
    if lead is not None:
      d = lead.dRel - 5.
      if 0. < d < -lead.vRel * 11. * 2. and lead.vRel < -1.:
        t = d / lead.vRel
        accel = -(lead.vRel / t) * self.speed_conv_to_clu
        accel *= 1.2

        if accel < 0.:
          target_speed = clu11_speed + accel
          target_speed = max(target_speed, self.min_set_speed_clu)
          return target_speed

    return 0

  def cal_curve_speed(self, sm, v_ego, frame):

    if frame % 20 == 0:
      md = sm['modelV2']
      if len(md.position.x) == TRAJECTORY_SIZE and len(md.position.y) == TRAJECTORY_SIZE:
        x = md.position.x
        y = md.position.y
        dy = np.gradient(y, x)
        d2y = np.gradient(dy, x)
        curv = d2y / (1 + dy ** 2) ** 1.5

        start = int(interp(v_ego, [10., 27.], [10, TRAJECTORY_SIZE-10]))
        curv = curv[start:min(start+10, TRAJECTORY_SIZE)]
        a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
        v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
        model_speed = np.mean(v_curvature) * 0.85

        if model_speed < v_ego:
          self.curve_speed_ms = float(max(model_speed, MIN_CURVE_SPEED))
        else:
          self.curve_speed_ms = 255.

        if np.isnan(self.curve_speed_ms):
          self.curve_speed_ms = 255.
      else:
        self.curve_speed_ms = 255.

  def cal_target_speed(self, CS, clu_speed, v_cruise_kph, cruise_btn_pressed):

    override_speed = -1

    if not self.long_control:
      if CS.gasPressed and self.sync_set_speed_while_gas_pressed and not cruise_btn_pressed:
        if clu_speed + SYNC_MARGIN > self.kph_to_clu(v_cruise_kph):
          set_speed = clip(clu_speed + SYNC_MARGIN, self.min_set_speed_clu, self.max_set_speed_clu)
          v_cruise_kph = int(round(set_speed * self.speed_conv_to_ms * CV.MS_TO_KPH))
          override_speed = v_cruise_kph

      self.target_speed = self.kph_to_clu(v_cruise_kph)
      if self.max_speed_clu > self.min_set_speed_clu:
        self.target_speed = clip(self.target_speed, self.min_set_speed_clu, self.max_speed_clu)

    elif CS.cruiseState.enabled:
      if CS.gasPressed and self.sync_set_speed_while_gas_pressed and not cruise_btn_pressed:
        if clu_speed + SYNC_MARGIN > self.kph_to_clu(v_cruise_kph):
          set_speed = clip(clu_speed + SYNC_MARGIN, self.min_set_speed_clu, self.max_set_speed_clu)
          self.target_speed = set_speed
          CruiseStateManager.instance().speed = set_speed * self.speed_conv_to_ms

    return override_speed

  def update_max_speed(self, max_speed, limited_curv):

    if not self.long_control or self.max_speed_clu <= 0:
      self.max_speed_clu = max_speed
    else:
      kp = 0.01 #if limited_curv else 0.01
      error = max_speed - self.max_speed_clu
      self.max_speed_clu = self.max_speed_clu + error * kp

  def get_button(self, current_set_speed):
    if self.target_speed < self.min_set_speed_clu:
      return Buttons.NONE
    error = self.target_speed - current_set_speed
    if abs(error) < 0.9:
      return Buttons.NONE
    return Buttons.RES_ACCEL if error > 0 else Buttons.SET_DECEL

  def initialize_v_cruise(self, v_ego, buttonEvents, v_cruise_last):
    for b in buttonEvents:
      # 250kph or above probably means we never had a set speed
      if b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) and v_cruise_last < 250:
        return v_cruise_last

    return int(round(clip(v_ego * CV.MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)))

  def update_v_cruise(self, CS, sm, enabled, is_metric, v_cruise_kph, v_cruise_kph_last):  # called by controlds's state_transition

    manage_button = not self.CP.openpilotLongitudinalControl or not self.CP.pcmCruise

    if CS.cruiseState.enabled:
      if manage_button:
        v_cruise_kph = self.update_cruise_button(v_cruise_kph, CS.buttonEvents, enabled, is_metric)
      else:
        v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
    else:
      v_cruise_kph = V_CRUISE_UNSET

    if self.prev_cruise_enabled != CS.cruiseState.enabled:
      self.prev_cruise_enabled = CS.cruiseState.enabled

      if CS.cruiseState.enabled:
        if not self.CP.pcmCruise:
          v_cruise_kph = self.initialize_v_cruise(CS.vEgo, CS.buttonEvents, v_cruise_kph_last)
        else:
          v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH

    self.real_set_speed_kph = v_cruise_kph

    if CS.cruiseState.enabled:
      clu_speed = CS.vEgoCluster * self.speed_conv_to_clu

      self.cal_max_speed(CS, sm, clu_speed, v_cruise_kph)
      self.cruise_speed_kph = float(clip(v_cruise_kph, V_CRUISE_MIN,
                                         self.max_speed_clu * self.speed_conv_to_ms * CV.MS_TO_KPH))

      if CruiseStateManager.instance().cruise_state_control:
        self.cruise_speed_kph = min(self.cruise_speed_kph, max(self.real_set_speed_kph, V_CRUISE_MIN_CRUISE_STATE))

    else:
      self.reset()

    return v_cruise_kph

  def update_can(self, enabled, CC, CS, sm, can_sends):

    new_v_cruise_kph = -1

    clu_speed = CS.vEgoCluster * self.speed_conv_to_clu
    ascc_enabled = enabled and CS.cruiseState.enabled and 1 < CS.cruiseState.speed < 255 and not CS.brakePressed

    btn_pressed = self.CI.CS.cruise_buttons[-1] != Buttons.NONE

    if not self.long_control:
      if not ascc_enabled or CS.cruiseState.standstill or btn_pressed:
        self.reset()
        self.wait_timer = max(self.alive_count_list) + max(self.wait_count_list)
        return new_v_cruise_kph

    if not ascc_enabled:
      self.reset()

    override_speed = self.cal_target_speed(CS, clu_speed, self.real_set_speed_kph, btn_pressed)
    if override_speed > 0:
      new_v_cruise_kph = override_speed

    if self.wait_timer > 0:
      self.wait_timer -= 1
    elif ascc_enabled and CS.vEgo > 0.1 and CruiseStateManager.instance().is_set_speed_spam_allowed(self.CP):
      if self.alive_timer == 0:
        current_set_speed_clu = int(round(CS.cruiseState.speed * self.speed_conv_to_clu))
        self.btn = self.get_button(current_set_speed_clu)
        self.alive_count = self.get_alive_count()

      if self.btn != Buttons.NONE:
        can = self.CI.create_buttons(self.btn)
        if can is not None:
          can_sends.append(can)

        self.alive_timer += 1

        if self.alive_timer >= self.alive_count:
          self.alive_timer = 0
          self.wait_timer = self.get_wait_count()
          self.btn = Buttons.NONE
      else:
        if self.long_control and self.target_speed >= self.min_set_speed_clu:
          self.target_speed = 0.
    else:
      if self.long_control:
        self.target_speed = 0.

    return new_v_cruise_kph

  def update_cruise_button(self, v_cruise_kph, buttonEvents, enabled, metric):

    if enabled:
      if self.button_count:
        self.button_count += 1
      for b in buttonEvents:
        if b.pressed and not self.button_count and (b.type == ButtonType.accelCruise or b.type == ButtonType.decelCruise):
          self.button_count = 1
          self.prev_button = b.type
        elif not b.pressed and self.button_count:
          if not self.long_pressed and b.type == ButtonType.accelCruise:
            v_cruise_kph += 1 if metric else 1 * CV.MPH_TO_KPH
          elif not self.long_pressed and b.type == ButtonType.decelCruise:
            v_cruise_kph -= 1 if metric else 1 * CV.MPH_TO_KPH
          self.long_pressed = False
          self.button_count = 0
      if self.button_count > 70:
        self.long_pressed = True
        V_CRUISE_DELTA = V_CRUISE_DELTA_KM if metric else V_CRUISE_DELTA_MI
        if self.prev_button == ButtonType.accelCruise:
          v_cruise_kph += V_CRUISE_DELTA - v_cruise_kph % V_CRUISE_DELTA
        elif self.prev_button == ButtonType.decelCruise:
          v_cruise_kph -= V_CRUISE_DELTA - -v_cruise_kph % V_CRUISE_DELTA
        self.button_count %= 70
      v_cruise_kph = clip(v_cruise_kph, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)

    return v_cruise_kph

  def update_message(self, c, CC, CS):

    CC.cruiseMaxSpeed = self.real_set_speed_kph
    CC.applyMaxSpeed = self.cruise_speed_kph
    CC.sccBus = c.CP.sccBus
    CC.steerRatio = c.VM.sR
    CC.steerActuatorDelay = ntune_common_get('steerActuatorDelay')

    if self.show_debug_message:
      actuators = c.last_actuators
      loc = c.LoC

      debug_text  = "Standstill: {}\n".format(CS.cruiseState.standstill)
      debug_text += "Long State: {}\n".format(actuators.longControlState)
      debug_text += "vEgo: {:.2f}/{:.2f}\n".format(CS.vEgo, CS.vEgo*3.6)

      debug_text += "vPid: {:.2f}/{:.2f}\n".format(loc.v_pid, loc.v_pid*3.6)
      debug_text += "PID: {:.2f}/{:.2f}/{:.2f}\n".format(loc.pid.p, loc.pid.i, loc.pid.f)

      debug_text += "Actuator Accel: {:.2f}\n".format(actuators.accel)
      debug_text += "Apply Accel: {:.2f}\n".format(CC.applyAccel)
      debug_text += "Stock Accel: {:.2f}\n".format(CS.aReqValue)

      lead_radar = c.sm['radarState'].leadOne
      lead_model = c.sm['modelV2'].leadsV3[0]

      radar_dist = lead_radar.dRel if lead_radar.status and lead_radar.radar else 0
      vision_dist = lead_model.x[0] - RADAR_TO_CAMERA if lead_model.prob > .5 else 0

      debug_text += "Lead: {:.1f}/{:.1f}/{:.1f}\n".format(radar_dist, vision_dist, (radar_dist - vision_dist))

      md = c.sm['modelV2']
      debug_text += "Lane: {:.2f}/{:.2f}, {:.2f}/{:.2f}".format(md.laneLineProbs[1], md.laneLineProbs[2],
                                                                md.laneLineStds[1], md.laneLineStds[2])

      CC.debugText = debug_text

