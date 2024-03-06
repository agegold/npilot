from collections import namedtuple
from dataclasses import dataclass, field

from cereal import car
from openpilot.selfdrive.car import AngleRateLimit, CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarInfo
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu

Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])


@dataclass
class TeslaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('tesla_powertrain', 'tesla_radar', chassis_dbc='tesla_can'))


class CAR(Platforms):
  AP1_MODELS = TeslaPlatformConfig(
    'TESLA AP1 MODEL S',
    CarInfo("Tesla AP1 Model S", "All"),
    CarSpecs(mass=2100., wheelbase=2.959, steerRatio=15.0)
  )
  AP2_MODELS = TeslaPlatformConfig(
    'TESLA AP2 MODEL S',
    CarInfo("Tesla AP2 Model S", "All"),
    AP1_MODELS.specs
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
      rx_offset=0x08,
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.adas, Ecu.electricBrakeBooster, Ecu.fwdRadar],
      rx_offset=0x10,
      bus=0,
    ),
  ]
)


class CANBUS:
  # Lateral harness
  chassis = 0
  radar = 1
  autopilot_chassis = 2

  # Longitudinal harness
  powertrain = 4
  private = 5
  autopilot_powertrain = 6

GEAR_MAP = {
  "DI_GEAR_INVALID": car.CarState.GearShifter.unknown,
  "DI_GEAR_P": car.CarState.GearShifter.park,
  "DI_GEAR_R": car.CarState.GearShifter.reverse,
  "DI_GEAR_N": car.CarState.GearShifter.neutral,
  "DI_GEAR_D": car.CarState.GearShifter.drive,
  "DI_GEAR_SNA": car.CarState.GearShifter.unknown,
}

DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]

# Make sure the message and addr is also in the CAN parser!
BUTTONS = [
  Button(car.CarState.ButtonEvent.Type.leftBlinker, "STW_ACTN_RQ", "TurnIndLvr_Stat", [1]),
  Button(car.CarState.ButtonEvent.Type.rightBlinker, "STW_ACTN_RQ", "TurnIndLvr_Stat", [2]),
  Button(car.CarState.ButtonEvent.Type.accelCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [4, 16]),
  Button(car.CarState.ButtonEvent.Type.decelCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [8, 32]),
  Button(car.CarState.ButtonEvent.Type.cancel, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [1]),
  Button(car.CarState.ButtonEvent.Type.resumeCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [2]),
]

class CarControllerParams:
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 1.6, .3])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 7.0, 0.8])
  JERK_LIMIT_MAX = 8
  JERK_LIMIT_MIN = -8
  ACCEL_TO_SPEED_MULTIPLIER = 3

  def __init__(self, CP):
    pass


CAR_INFO = CAR.create_carinfo_map()
DBC = CAR.create_dbc_map()
