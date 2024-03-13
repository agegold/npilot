from dataclasses import dataclass, field
from enum import IntFlag

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarHarness, CarInfo, CarParts
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  STEER_MAX = 800                # theoretical max_steer 2047
  STEER_DELTA_UP = 10             # torque increase per refresh
  STEER_DELTA_DOWN = 25           # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 1     # weight driver torque
  STEER_DRIVER_FACTOR = 1         # from dbc
  STEER_ERROR_MAX = 350           # max delta between torque cmd and torque motor
  STEER_STEP = 1  # 100 Hz

  def __init__(self, CP):
    pass


@dataclass
class MazdaCarInfo(CarInfo):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


class MazdaFlags(IntFlag):
  # Static flags
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('mazda_2017', None))
  flags: int = MazdaFlags.GEN1


class CAR(Platforms):
  CX5 = MazdaPlatformConfig(
    "MAZDA CX-5",
    MazdaCarInfo("Mazda CX-5 2017-21"),
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5)
  )
  CX9 = MazdaPlatformConfig(
    "MAZDA CX-9",
    MazdaCarInfo("Mazda CX-9 2016-20"),
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=3.1, steerRatio=17.6)
  )
  MAZDA3 = MazdaPlatformConfig(
    "MAZDA 3",
    MazdaCarInfo("Mazda 3 2017-18"),
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0)
  )
  MAZDA6 = MazdaPlatformConfig(
    "MAZDA 6",
    MazdaCarInfo("Mazda 6 2017-20"),
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5)
  )
  CX9_2021 = MazdaPlatformConfig(
    "MAZDA CX-9 2021",
    MazdaCarInfo("Mazda CX-9 2021-23", video_link="https://youtu.be/dA3duO4a0O4"),
    CX9.specs
  )
  CX5_2022 = MazdaPlatformConfig(
    "MAZDA CX-5 2022",
    MazdaCarInfo("Mazda CX-5 2022-24"),
    CX5.specs,
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
