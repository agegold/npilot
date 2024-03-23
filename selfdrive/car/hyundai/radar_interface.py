import math

from cereal import car
from common.numpy_fast import clip
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from openpilot.selfdrive.car.hyundai.values import DBC, CANFD_CAR
from openpilot.selfdrive.controls.neokii.cruise_state_manager import is_radar_disabler

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/

def get_radar_can_parser(CP):

  if CP.carFingerprint in CANFD_CAR or is_radar_disabler(CP):

    if DBC[CP.carFingerprint]['radar'] is None:
      return None

    messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
    return CANParser(DBC[CP.carFingerprint]['radar'], messages, 1)

  else:
    messages = [
      ("SCC11", 50),
    ]
    return CANParser(DBC[CP.carFingerprint]['pt'], messages, CP.sccBus)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.new_radar = is_radar_disabler(CP)
    self.updated_messages = set()
    self.trigger_msg = 0x420 if not self.new_radar else RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    self.track_id = 0
    self.prev_dist = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    if self.rcp is None:
      return ret

    errors = []

    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    if self.new_radar:

      for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
        msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

        if addr not in self.pts:
          self.pts[addr] = car.RadarData.RadarPoint.new_message()
          self.pts[addr].trackId = self.track_id
          self.track_id += 1

        valid = msg['STATE'] in (3, 4)
        if valid:
          azimuth = math.radians(msg['AZIMUTH'])
          self.pts[addr].measured = True
          self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
          self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
          self.pts[addr].vRel = msg['REL_SPEED']
          self.pts[addr].aRel = msg['REL_ACCEL']
          self.pts[addr].yvRel = float('nan')

        else:
          del self.pts[addr]

      ret.points = list(self.pts.values())
      return ret

    else:
      cpt = self.rcp.vl

      valid = cpt["SCC11"]['ACC_ObjStatus']
      if valid:
        dist = cpt["SCC11"]['ACC_ObjDist']
        if abs(dist - self.prev_dist) > clip(dist/20., 1., 5.):
          self.pts.clear()

        self.prev_dist = dist

        target_id = 0
        if target_id not in self.pts:
          self.pts[target_id] = car.RadarData.RadarPoint.new_message()
          self.pts[target_id].trackId = self.track_id
          self.track_id += 1

        self.pts[target_id].dRel = dist  # from front of car
        self.pts[target_id].yRel = -cpt["SCC11"]['ACC_ObjLatPos']  # in car frame's y axis, left is negative
        self.pts[target_id].vRel = cpt["SCC11"]['ACC_ObjRelSpd']
        self.pts[target_id].aRel = float('nan')
        self.pts[target_id].yvRel = float('nan')
        self.pts[target_id].measured = True
      else:
        self.pts.clear()

      ret.points = list(self.pts.values())
      return ret
