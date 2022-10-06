import copy
from collections import deque

from cereal import car
from common.numpy_fast import interp
from selfdrive.car.hyundai.values import DBC, FEATURES, CANFD_CAR, EV_CAR, HYBRID_CAR, Buttons, CarControllerParams, \
  EV_HYBRID_CAR, CAR, HyundaiFlags
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.conversions import Conversions as CV
from common.params import Params

PREV_BUTTON_SAMPLES = 8

GearShifter = car.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    self.cruise_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.main_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)

    if CP.carFingerprint in CANFD_CAR:
      self.shifter_values = can_define.dv["GEAR_SHIFTER"]["GEAR"]
    elif self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    else:  # preferred and elect gear methods use same definition
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    #Auto detection for setup
    self.no_radar = CP.sccBus == -1
    self.mdps_bus = CP.mdpsBus
    self.sas_bus = CP.sasBus
    self.scc_bus = CP.sccBus
    self.has_scc13 = CP.hasScc13 or CP.carFingerprint in FEATURES["has_scc13"]
    self.has_scc14 = CP.hasScc14 or CP.carFingerprint in FEATURES["has_scc14"]
    self.has_lfa_hda = CP.hasLfaHda
    self.leftBlinker = False
    self.rightBlinker = False
    self.mdps_error_cnt = 0
    self.cruise_unavail_cnt = 0

    self.apply_steer = 0.
    self.buttons_counter = 0

    self.params = CarControllerParams(CP)

    # scc smoother
    self.acc_mode = False

    self.use_cluster_speed = Params().get_bool('UseClusterSpeed')
    self.long_control_enabled = Params().get_bool('LongControlEnabled')

  def update(self, cp, cp2, cp_cam):

    self.prev_left_blinker = self.leftBlinker
    self.prev_right_blinker = self.rightBlinker
    self.prev_cruise_buttons = self.cruise_buttons[-1]
    self.prev_main_button = self.main_buttons[-1]

    if self.CP.carFingerprint in CANFD_CAR:
      return self.update_canfd(cp, cp_cam)

    cp_mdps = cp2 if self.mdps_bus else cp
    cp_sas = cp2 if self.sas_bus else cp
    cp_scc = cp2 if self.scc_bus == 1 else cp_cam if self.scc_bus == 2 else cp

    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["CGW1"]["CF_Gway_DrvDrSw"], cp.vl["CGW1"]["CF_Gway_AstDrSw"],
                        cp.vl["CGW2"]["CF_Gway_RLDrSw"], cp.vl["CGW2"]["CF_Gway_RRDrSw"]])

    ret.seatbeltUnlatched = cp.vl["CGW1"]["CF_Gway_DrvSeatBeltSw"] == 0

    self.is_set_speed_in_mph = bool(cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"])
    self.speed_conv_to_ms = CV.MPH_TO_MS if self.is_set_speed_in_mph else CV.KPH_TO_MS

    cluSpeed = cp.vl["CLU11"]["CF_Clu_Vanz"]
    decimal = cp.vl["CLU11"]["CF_Clu_VanzDecimal"]
    if 0. < decimal < 0.5:
      cluSpeed += decimal

    ret.vEgoCluster = cluSpeed * self.speed_conv_to_ms

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHL_SPD11"]["WHL_SPD_FL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_FR"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RR"],
    )

    vEgoRawClu = cluSpeed * self.speed_conv_to_ms
    vEgoClu, aEgoClu = self.update_clu_speed_kf(vEgoRawClu)

    vEgoRawWheel = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    vEgoRawWheel = interp(vEgoRawWheel, [0., 10.], [(vEgoRawWheel + vEgoRawClu) / 2., vEgoRawWheel])
    vEgoWheel, aEgoWheel = self.update_speed_kf(vEgoRawWheel)

    if self.use_cluster_speed:
      ret.vEgoRaw = vEgoRawClu
      ret.vEgo = vEgoClu
      ret.aEgo = aEgoClu
    else:
      ret.vEgoRaw = vEgoRawWheel
      ret.vEgo = vEgoWheel
      ret.aEgo = aEgoWheel

    ret.vCluRatio = (vEgoWheel / vEgoClu) if (vEgoClu > 3. and vEgoWheel > 3.) else 1.0
    ret.aBasis = cp.vl["TCS13"]["aBasis"]

    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = cp_sas.vl["SAS11"]["SAS_Angle"]
    ret.steeringRateDeg = cp_sas.vl["SAS11"]["SAS_Speed"]
    ret.yawRate = cp.vl["ESP12"]["YAW_RATE"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["CGW1"]["CF_Gway_TurnSigLh"],
                                                            cp.vl["CGW1"]["CF_Gway_TurnSigRh"])
    ret.steeringTorque = cp_mdps.vl["MDPS12"]["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = cp_mdps.vl["MDPS12"]["CR_Mdps_OutTq"] / 10.  # scale to Nm
    ret.steeringPressed = abs(ret.steeringTorque) > self.params.STEER_THRESHOLD

    if not ret.standstill and cp_mdps.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0:
      self.mdps_error_cnt += 1
    else:
      self.mdps_error_cnt = 0

    ret.steerFaultTemporary = self.mdps_error_cnt > 50

    if self.CP.enableAutoHold:
      ret.autoHold = cp.vl["ESP11"]["AVH_STAT"]

    # cruise state
    ret.cruiseState.enabled = (cp_scc.vl["SCC12"]["ACCMode"] != 0) if not self.no_radar else \
                                      cp.vl["LVR12"]["CF_Lvr_CruiseSet"] != 0
    ret.cruiseState.available = (cp_scc.vl["SCC11"]["MainMode_ACC"] != 0) if not self.no_radar else \
                                      cp.vl["EMS16"]["CRUISE_LAMP_M"] != 0
    ret.cruiseState.standstill = cp_scc.vl["SCC11"]["SCCInfoDisplay"] == 4. if not self.no_radar else False

    ret.cruiseState.enabledAcc = ret.cruiseState.enabled

    if ret.cruiseState.enabled:
      ret.cruiseState.speed = cp_scc.vl["SCC11"]["VSetDis"] * self.speed_conv_to_ms if not self.no_radar else \
                                         cp.vl["LVR12"]["CF_Lvr_CruiseSet"] * self.speed_conv_to_ms
    else:
      ret.cruiseState.speed = 0

    self.cruise_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"])
    self.main_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwMain"])

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["TCS13"]["DriverBraking"] != 0
    ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2  # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
    ret.parkingBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
    #ret.parkingBrake = cp.vl["CGW1"]["CF_Gway_ParkBrakeSw"]

    # TODO: Check this
    ret.brakeLights = bool(cp.vl["TCS13"]["BrakeLight"] or ret.brakePressed)
    ret.gasPressed = cp.vl["TCS13"]["DriverOverride"] == 1

    if self.CP.carFingerprint in EV_HYBRID_CAR:
      if self.CP.carFingerprint in HYBRID_CAR:
        ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
      else:
        ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.

    if self.CP.hasEms:
      ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])

    # TODO: refactor gear parsing in function
    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      gear = cp.vl["TCU12"]["CUR_GR"]
    elif self.CP.carFingerprint in FEATURES["use_elect_gears"]:
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    if self.CP.carFingerprint in FEATURES["use_fca"]:
      ret.stockAeb = cp.vl["FCA11"]["FCA_CmdAct"] != 0
      ret.stockFcw = cp.vl["FCA11"]["CF_VSM_Warn"] == 2
    else:
      ret.stockAeb = cp.vl["SCC12"]["AEB_CmdAct"] != 0
      ret.stockFcw = cp.vl["SCC12"]["CF_VSM_Warn"] == 2

    # Blind Spot Detection and Lane Change Assist signals
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0
    else:
      ret.leftBlindspot = False
      ret.rightBlindspot = False

    # save the entire LKAS11, CLU11, SCC12 and MDPS12
    self.lkas11 = cp_cam.vl["LKAS11"]
    self.clu11 = cp.vl["CLU11"]
    self.scc11 = cp_scc.vl["SCC11"]
    self.scc12 = cp_scc.vl["SCC12"]
    self.mdps12 = cp_mdps.vl["MDPS12"]
    self.lfahda_mfc = cp_cam.vl["LFAHDA_MFC"]
    self.steer_state = cp_mdps.vl["MDPS12"]["CF_Mdps_ToiActive"] #0 NOT ACTIVE, 1 ACTIVE
    self.cruise_unavail_cnt += 1 if cp.vl["TCS13"]["CF_VSM_Avail"] != 1 and cp.vl["TCS13"]["ACCEnable"] != 0 else -self.cruise_unavail_cnt
    self.cruise_unavail = self.cruise_unavail_cnt > 100

    self.lead_distance = cp_scc.vl["SCC11"]["ACC_ObjDist"] if not self.no_radar else 0
    if self.has_scc13:
      self.scc13 = cp_scc.vl["SCC13"]
    if self.has_scc14:
      self.scc14 = cp_scc.vl["SCC14"]

    # scc smoother
    self.acc_mode = cp_scc.vl["SCC12"]["ACCMode"] != 0
    ret.cruiseGap = cp_scc.vl["SCC11"]["TauGapSet"] if not self.no_radar else 1

    tpms_unit = cp.vl["TPMS11"]["UNIT"] * 0.725 if int(cp.vl["TPMS11"]["UNIT"]) > 0 else 1.
    ret.tpms.fl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FL"]
    ret.tpms.fr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FR"]
    ret.tpms.rl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RL"]
    ret.tpms.rr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RR"]

    return ret

  def update_canfd(self, cp, cp_cam):
    ret = car.CarState.new_message()

    if self.CP.flags & HyundaiFlags.CANFD_HDA2:
      ret.gas = cp.vl["ACCELERATOR"]["ACCELERATOR_PEDAL"] / 255.
    else:
      ret.gas = cp.vl["ACCELERATOR_ALT"]["ACCELERATOR_PEDAL"] / 1023.
    ret.gasPressed = ret.gas > 1e-5
    ret.brakePressed = cp.vl["BRAKE"]["BRAKE_PRESSED"] == 1

    ret.doorOpen = cp.vl["DOORS_SEATBELTS"]["DRIVER_DOOR_OPEN"] == 1
    ret.seatbeltUnlatched = cp.vl["DOORS_SEATBELTS"]["DRIVER_SEATBELT_LATCHED"] == 0

    gear = cp.vl["GEAR_SHIFTER"]["GEAR"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    # TODO: figure out positions
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_1"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_2"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_3"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_4"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringRateDeg = cp.vl["STEERING_SENSORS"]["STEERING_RATE"]
    ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEERING_ANGLE"] * -1
    ret.steeringTorque = cp.vl["MDPS"]["STEERING_COL_TORQUE"]
    ret.steeringTorqueEps = cp.vl["MDPS"]["STEERING_OUT_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > self.params.STEER_THRESHOLD

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["BLINKERS"]["LEFT_LAMP"],
                                                                      cp.vl["BLINKERS"]["RIGHT_LAMP"])

    ret.cruiseState.available = True
    ret.cruiseState.enabled = cp.vl["SCC1"]["CRUISE_ACTIVE"] == 1
    cp_cruise_info = cp if self.CP.flags & HyundaiFlags.CANFD_HDA2 else cp_cam
    speed_factor = CV.MPH_TO_MS if cp.vl["CLUSTER_INFO"]["DISTANCE_UNIT"] == 1 else CV.KPH_TO_MS
    ret.cruiseState.speed = cp_cruise_info.vl["CRUISE_INFO"]["SET_SPEED"] * speed_factor
    ret.cruiseState.standstill = cp_cruise_info.vl["CRUISE_INFO"]["CRUISE_STANDSTILL"] == 1

    cruise_btn_msg = "CRUISE_BUTTONS_ALT" if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else "CRUISE_BUTTONS"
    self.cruise_buttons.extend(cp.vl_all[cruise_btn_msg]["CRUISE_BUTTONS"])
    self.main_buttons.extend(cp.vl_all[cruise_btn_msg]["ADAPTIVE_CRUISE_MAIN_BTN"])
    self.buttons_counter = cp.vl[cruise_btn_msg]["COUNTER"]
    self.cruise_info_copy = copy.copy(cp_cruise_info.vl["CRUISE_INFO"])

    if self.CP.flags & HyundaiFlags.CANFD_HDA2:
      self.cam_0x2a4 = copy.copy(cp_cam.vl["CAM_0x2a4"])

    # scc smoother
    self.is_set_speed_in_mph = cp.vl["CLUSTER_INFO"]["DISTANCE_UNIT"] == 1
    self.speed_conv_to_ms = CV.MPH_TO_MS if self.is_set_speed_in_mph else CV.KPH_TO_MS
    self.acc_mode = cp.vl["SCC1"]["CRUISE_ACTIVE"] == 1

    # TODO
    ret.vEgoCluster = ret.vEgoRaw  #cluSpeed * self.speed_conv_to_ms

    return ret

  @staticmethod
  def get_can_parser(CP):
    if CP.carFingerprint in CANFD_CAR:
      return CarState.get_can_parser_canfd(CP)

    signals = [
      # sig_name, sig_address
      ("WHL_SPD_FL", "WHL_SPD11"),
      ("WHL_SPD_FR", "WHL_SPD11"),
      ("WHL_SPD_RL", "WHL_SPD11"),
      ("WHL_SPD_RR", "WHL_SPD11"),

      ("YAW_RATE", "ESP12"),

      ("CF_Gway_DrvSeatBeltInd", "CGW4"),

      ("CF_Gway_DrvSeatBeltSw", "CGW1"),
      ("CF_Gway_DrvDrSw", "CGW1"),       # Driver Door
      ("CF_Gway_AstDrSw", "CGW1"),       # Passenger door
      ("CF_Gway_RLDrSw", "CGW2"),        # Rear reft door
      ("CF_Gway_RRDrSw", "CGW2"),        # Rear right door
      ("CF_Gway_TurnSigLh", "CGW1"),
      ("CF_Gway_TurnSigRh", "CGW1"),
      ("CF_Gway_ParkBrakeSw", "CGW1"),   # Parking Brake

      ("CYL_PRES", "ESP12"),

      ("CF_Clu_CruiseSwState", "CLU11"),
      ("CF_Clu_CruiseSwMain", "CLU11"),
      ("CF_Clu_SldMainSW", "CLU11"),
      ("CF_Clu_ParityBit1", "CLU11"),
      ("CF_Clu_VanzDecimal" , "CLU11"),
      ("CF_Clu_Vanz", "CLU11"),
      ("CF_Clu_SPEED_UNIT", "CLU11"),
      ("CF_Clu_DetentOut", "CLU11"),
      ("CF_Clu_RheostatLevel", "CLU11"),
      ("CF_Clu_CluInfo", "CLU11"),
      ("CF_Clu_AmpInfo", "CLU11"),
      ("CF_Clu_AliveCnt1", "CLU11"),

      ("ACCEnable", "TCS13"),
      ("BrakeLight", "TCS13"),
      ("aBasis", "TCS13"),
      ("DriverBraking", "TCS13"),
      ("PBRAKE_ACT", "TCS13"),
      ("DriverOverride", "TCS13"),
      ("CF_VSM_Avail", "TCS13"),

      ("ESC_Off_Step", "TCS15"),
      ("AVH_LAMP", "TCS15"),

      #("CF_Lvr_GearInf", "LVR11"),        # Transmission Gear (0 = N or P, 1-8 = Fwd, 14 = Rev)

      ("MainMode_ACC", "SCC11"),
      ("SCCInfoDisplay", "SCC11"),
      ("AliveCounterACC", "SCC11"),
      ("VSetDis", "SCC11"),
      ("ObjValid", "SCC11"),
      ("DriverAlertDisplay", "SCC11"),
      ("TauGapSet", "SCC11"),
      ("ACC_ObjStatus", "SCC11"),
      ("ACC_ObjLatPos", "SCC11"),
      ("ACC_ObjDist", "SCC11"), #TK211X value is 204.6
      ("ACC_ObjRelSpd", "SCC11"),
      ("Navi_SCC_Curve_Status", "SCC11"),
      ("Navi_SCC_Curve_Act", "SCC11"),
      ("Navi_SCC_Camera_Act", "SCC11"),
      ("Navi_SCC_Camera_Status", "SCC11"),

      ("ACCMode", "SCC12"),
      ("CF_VSM_Prefill", "SCC12"),
      ("CF_VSM_DecCmdAct", "SCC12"),
      ("CF_VSM_HBACmd", "SCC12"),
      ("CF_VSM_Warn", "SCC12"),
      ("CF_VSM_Stat", "SCC12"),
      ("CF_VSM_BeltCmd", "SCC12"),
      ("ACCFailInfo", "SCC12"),
      ("StopReq", "SCC12"),
      ("CR_VSM_DecCmd", "SCC12"),
      ("aReqRaw", "SCC12"), #aReqMax
      ("TakeOverReq", "SCC12"),
      ("PreFill", "SCC12"),
      ("aReqValue", "SCC12"), #aReqMin
      ("CF_VSM_ConfMode", "SCC12"),
      ("AEB_Failinfo", "SCC12"),
      ("AEB_Status", "SCC12"),
      ("AEB_CmdAct", "SCC12"),
      ("AEB_StopReq", "SCC12"),
      ("CR_VSM_Alive", "SCC12"),
      ("CR_VSM_ChkSum", "SCC12"),

      ("SCCDrvModeRValue", "SCC13"),
      ("SCC_Equip", "SCC13"),
      ("AebDrvSetStatus", "SCC13"),

      ("JerkUpperLimit", "SCC14"),
      ("JerkLowerLimit", "SCC14"),
      ("SCCMode2", "SCC14"),
      ("ComfortBandUpper", "SCC14"),
      ("ComfortBandLower", "SCC14"),

      ("UNIT", "TPMS11"),
      ("PRESSURE_FL", "TPMS11"),
      ("PRESSURE_FR", "TPMS11"),
      ("PRESSURE_RL", "TPMS11"),
      ("PRESSURE_RR", "TPMS11"),
    ]

    checks = [
      # address, frequency
      ("TCS13", 50),
      ("TCS15", 10),
      ("CLU11", 50),
      ("ESP12", 100),
      ("CGW1", 10),
      ("CGW2", 5),
      ("CGW4", 5),
      ("WHL_SPD11", 50),
    ]

    if CP.sccBus == 0 and CP.pcmCruise:
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
    if CP.mdpsBus == 0:
      signals += [
        ("CR_Mdps_StrColTq", "MDPS12"),
        ("CF_Mdps_Def", "MDPS12"),
        ("CF_Mdps_ToiActive", "MDPS12"),
        ("CF_Mdps_ToiUnavail", "MDPS12"),
        ("CF_Mdps_ToiFlt", "MDPS12"),
        ("CF_Mdps_MsgCount2", "MDPS12"),
        ("CF_Mdps_Chksum2", "MDPS12"),
        ("CF_Mdps_SErr", "MDPS12"),
        ("CR_Mdps_StrTq", "MDPS12"),
        ("CF_Mdps_FailStat", "MDPS12"),
        ("CR_Mdps_OutTq", "MDPS12")
      ]
      checks += [
        ("MDPS12", 50)
      ]
    if CP.sasBus == 0:
      signals += [
        ("SAS_Angle", "SAS11"),
        ("SAS_Speed", "SAS11"),
      ]
      checks += [
        ("SAS11", 100)
      ]
    if CP.sccBus == -1:
      signals += [
        ("CRUISE_LAMP_M", "EMS16"),
        ("CF_Lvr_CruiseSet", "LVR12"),
    ]
    if CP.carFingerprint in FEATURES["use_cluster_gears"]:
      signals += [
        ("CF_Clu_Gear", "CLU15"),
      ]
    elif CP.carFingerprint in FEATURES["use_tcu_gears"]:
      signals += [
        ("CUR_GR", "TCU12"),
      ]
    elif CP.carFingerprint in FEATURES["use_elect_gears"]:
      signals += [
        ("Elect_Gear_Shifter", "ELECT_GEAR"),
    ]
    else:
      signals += [
        ("CF_Lvr_Gear","LVR12"),
      ]

    if CP.carFingerprint in EV_HYBRID_CAR:
      if CP.carFingerprint in HYBRID_CAR:
        signals += [
          ("CR_Vcu_AccPedDep_Pos", "E_EMS11")
        ]
      else:
        signals += [
          ("Accel_Pedal_Pos", "E_EMS11"),
        ]
      checks += [
        ("E_EMS11", 50),
      ]

    else:
      signals += [
        ("PV_AV_CAN", "EMS12"),
        ("CF_Ems_AclAct", "EMS16"),
      ]
      checks += [
        ("EMS12", 100),
        ("EMS16", 100),
      ]

    if CP.carFingerprint in FEATURES["use_fca"]:
      signals += [
        ("FCA_CmdAct", "FCA11"),
        ("CF_VSM_Warn", "FCA11"),
      ]

      if not CP.openpilotLongitudinalControl:
        checks += [("FCA11", 50)]

    if CP.carFingerprint in [CAR.SANTA_FE, CAR.SANTA_FE_2022]:
      checks.remove(("TCS13", 50))

    if CP.enableBsm:
      signals += [
        ("CF_Lca_IndLeft", "LCA11"),
        ("CF_Lca_IndRight", "LCA11"),
      ]
      checks += [("LCA11", 50)]

    if CP.enableAutoHold:
      signals += [
        ("AVH_STAT", "ESP11"),
        ("LDM_STAT", "ESP11"),
      ]
      checks += [("ESP11", 50)]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0, enforce_checks=False)

  @staticmethod
  def get_can2_parser(CP):
    if CP.carFingerprint in CANFD_CAR:
      return None

    signals = []
    checks = []
    if CP.mdpsBus == 1:
      signals += [
        ("CR_Mdps_StrColTq", "MDPS12"),
        ("CF_Mdps_Def", "MDPS12"),
        ("CF_Mdps_ToiActive", "MDPS12"),
        ("CF_Mdps_ToiUnavail", "MDPS12"),
        ("CF_Mdps_ToiFlt", "MDPS12"),
        ("CF_Mdps_MsgCount2", "MDPS12"),
        ("CF_Mdps_Chksum2", "MDPS12"),
        ("CF_Mdps_SErr", "MDPS12"),
        ("CR_Mdps_StrTq", "MDPS12"),
        ("CF_Mdps_FailStat", "MDPS12"),
        ("CR_Mdps_OutTq", "MDPS12")
      ]
      checks += [
        ("MDPS12", 50)
      ]

    if CP.sasBus == 1:
      signals += [
        ("SAS_Angle", "SAS11"),
        ("SAS_Speed", "SAS11"),
      ]
      checks += [
        ("SAS11", 100)
      ]
    if CP.sccBus == 1:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("AliveCounterACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("ObjValid", "SCC11"),
        ("DriverAlertDisplay", "SCC11"),
        ("TauGapSet", "SCC11"),
        ("ACC_ObjStatus", "SCC11"),
        ("ACC_ObjLatPos", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("ACC_ObjRelSpd", "SCC11"),
        ("Navi_SCC_Curve_Status", "SCC11"),
        ("Navi_SCC_Curve_Act", "SCC11"),
        ("Navi_SCC_Camera_Act", "SCC11"),
        ("Navi_SCC_Camera_Status", "SCC11"),


        ("ACCMode", "SCC12"),
        ("CF_VSM_Prefill", "SCC12"),
        ("CF_VSM_DecCmdAct", "SCC12"),
        ("CF_VSM_HBACmd", "SCC12"),
        ("CF_VSM_Warn", "SCC12"),
        ("CF_VSM_Stat", "SCC12"),
        ("CF_VSM_BeltCmd", "SCC12"),
        ("ACCFailInfo", "SCC12"),
        ("StopReq", "SCC12"),
        ("CR_VSM_DecCmd", "SCC12"),
        ("aReqRaw", "SCC12"), #aReqMax
        ("TakeOverReq", "SCC12"),
        ("PreFill", "SCC12"),
        ("aReqValue", "SCC12"), #aReqMin
        ("CF_VSM_ConfMode", "SCC12"),
        ("AEB_Failinfo", "SCC12"),
        ("AEB_Status", "SCC12"),
        ("AEB_CmdAct", "SCC12"),
        ("AEB_StopReq", "SCC12"),
        ("CR_VSM_Alive", "SCC12"),
        ("CR_VSM_ChkSum", "SCC12"),

        ("SCCDrvModeRValue", "SCC13"),
        ("SCC_Equip", "SCC13"),
        ("AebDrvSetStatus", "SCC13"),

        ("JerkUpperLimit", "SCC14"),
        ("JerkLowerLimit", "SCC14"),
        ("SCCMode2", "SCC14"),
        ("ComfortBandUpper", "SCC14"),
        ("ComfortBandLower", "SCC14"),

      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 1, enforce_checks=False)

  @staticmethod
  def get_cam_can_parser(CP):
    if CP.carFingerprint in CANFD_CAR:
      return CarState.get_cam_can_parser_canfd(CP)

    signals = [
      # sig_name, sig_address, default
      ("CF_Lkas_LdwsActivemode", "LKAS11"),
      ("CF_Lkas_LdwsSysState", "LKAS11"),
      ("CF_Lkas_SysWarning", "LKAS11"),
      ("CF_Lkas_LdwsLHWarning", "LKAS11"),
      ("CF_Lkas_LdwsRHWarning", "LKAS11"),
      ("CF_Lkas_HbaLamp", "LKAS11"),
      ("CF_Lkas_FcwBasReq", "LKAS11"),
      ("CF_Lkas_ToiFlt", "LKAS11"),
      ("CF_Lkas_HbaSysState", "LKAS11"),
      ("CF_Lkas_FcwOpt", "LKAS11"),
      ("CF_Lkas_HbaOpt", "LKAS11"),
      ("CF_Lkas_FcwSysState", "LKAS11"),
      ("CF_Lkas_FcwCollisionWarning", "LKAS11"),
      ("CF_Lkas_MsgCount", "LKAS11"),
      ("CF_Lkas_FusionState", "LKAS11"),
      ("CF_Lkas_FcwOpt_USM", "LKAS11"),
      ("CF_Lkas_LdwsOpt_USM", "LKAS11"),
    ]

    checks = [
      ("LKAS11", 100)
    ]
    if CP.sccBus == 2:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("AliveCounterACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("ObjValid", "SCC11"),
        ("DriverAlertDisplay", "SCC11"),
        ("TauGapSet", "SCC11"),
        ("ACC_ObjStatus", "SCC11"),
        ("ACC_ObjLatPos", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("ACC_ObjRelSpd", "SCC11"),
        ("Navi_SCC_Curve_Status", "SCC11"),
        ("Navi_SCC_Curve_Act", "SCC11"),
        ("Navi_SCC_Camera_Act", "SCC11"),
        ("Navi_SCC_Camera_Status", "SCC11"),

        ("ACCMode", "SCC12"),
        ("CF_VSM_Prefill", "SCC12"),
        ("CF_VSM_DecCmdAct", "SCC12"),
        ("CF_VSM_HBACmd", "SCC12"),
        ("CF_VSM_Warn", "SCC12"),
        ("CF_VSM_Stat", "SCC12"),
        ("CF_VSM_BeltCmd", "SCC12"),
        ("ACCFailInfo", "SCC12"),
        ("StopReq", "SCC12"),
        ("CR_VSM_DecCmd", "SCC12"),
        ("aReqRaw", "SCC12"), #aReqMax
        ("TakeOverReq", "SCC12"),
        ("PreFill", "SCC12"),
        ("aReqValue", "SCC12"), #aReqMin
        ("CF_VSM_ConfMode", "SCC12"),
        ("AEB_Failinfo", "SCC12"),
        ("AEB_Status", "SCC12"),
        ("AEB_CmdAct", "SCC12"),
        ("AEB_StopReq", "SCC12"),
        ("CR_VSM_Alive", "SCC12"),
        ("CR_VSM_ChkSum", "SCC12"),

        ("SCCDrvModeRValue", "SCC13"),
        ("SCC_Equip", "SCC13"),
        ("AebDrvSetStatus", "SCC13"),

        ("JerkUpperLimit", "SCC14"),
        ("JerkLowerLimit", "SCC14"),
        ("SCCMode2", "SCC14"),
        ("ComfortBandUpper", "SCC14"),
        ("ComfortBandLower", "SCC14"),
      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]

      if CP.hasLfaHda:
        signals += [
          ("HDA_USM", "LFAHDA_MFC"),
          ("HDA_Active", "LFAHDA_MFC"),
          ("HDA_Icon_State", "LFAHDA_MFC"),
          ("HDA_LdwSysState", "LFAHDA_MFC"),
          ("HDA_Icon_Wheel", "LFAHDA_MFC"),
        ]
        checks += [("LFAHDA_MFC", 20)]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2, enforce_checks=False)

  @staticmethod
  def get_can_parser_canfd(CP):

    cruise_btn_msg = "CRUISE_BUTTONS_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else "CRUISE_BUTTONS"
    signals = [
      ("WHEEL_SPEED_1", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_2", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_3", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_4", "WHEEL_SPEEDS"),

      ("GEAR", "GEAR_SHIFTER"),
      ("BRAKE_PRESSED", "BRAKE"),

      ("STEERING_RATE", "STEERING_SENSORS"),
      ("STEERING_ANGLE", "STEERING_SENSORS"),
      ("STEERING_COL_TORQUE", "MDPS"),
      ("STEERING_OUT_TORQUE", "MDPS"),

      ("CRUISE_ACTIVE", "SCC1"),
      ("COUNTER", cruise_btn_msg),
      ("CRUISE_BUTTONS", cruise_btn_msg),
      ("ADAPTIVE_CRUISE_MAIN_BTN", cruise_btn_msg),

      ("DISTANCE_UNIT", "CLUSTER_INFO"),

      ("LEFT_LAMP", "BLINKERS"),
      ("RIGHT_LAMP", "BLINKERS"),

      ("DRIVER_DOOR_OPEN", "DOORS_SEATBELTS"),
      ("DRIVER_SEATBELT_LATCHED", "DOORS_SEATBELTS"),
    ]

    checks = [
      ("WHEEL_SPEEDS", 100),
      ("GEAR_SHIFTER", 100),
      ("BRAKE", 100),
      ("STEERING_SENSORS", 100),
      ("MDPS", 100),
      ("SCC1", 50),
      (cruise_btn_msg, 50),
      ("CLUSTER_INFO", 4),
      ("BLINKERS", 4),
      ("DOORS_SEATBELTS", 4),
    ]

    if CP.flags & HyundaiFlags.CANFD_HDA2:
      signals += [
        ("ACCELERATOR_PEDAL", "ACCELERATOR"),
        ("GEAR", "ACCELERATOR"),
        ("SET_SPEED", "CRUISE_INFO"),
        ("CRUISE_STANDSTILL", "CRUISE_INFO"),
      ]
      checks += [
        ("CRUISE_INFO", 50),
        ("ACCELERATOR", 100),
      ]
    else:
      signals += [
        ("ACCELERATOR_PEDAL", "ACCELERATOR_ALT"),
      ]
      checks += [
        ("ACCELERATOR_ALT", 100),
      ]

    bus = 5 if CP.flags & HyundaiFlags.CANFD_HDA2 else 4
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, bus)

  @staticmethod
  def get_cam_can_parser_canfd(CP):
    if CP.flags & HyundaiFlags.CANFD_HDA2:
      signals = [(f"BYTE{i}", "CAM_0x2a4") for i in range(3, 24)]
      checks = [("CAM_0x2a4", 20)]
    else:
      signals = [
        ("COUNTER", "CRUISE_INFO"),
        ("NEW_SIGNAL_1", "CRUISE_INFO"),
        ("CRUISE_MAIN", "CRUISE_INFO"),
        ("CRUISE_STATUS", "CRUISE_INFO"),
        ("CRUISE_INACTIVE", "CRUISE_INFO"),
        ("NEW_SIGNAL_2", "CRUISE_INFO"),
        ("CRUISE_STANDSTILL", "CRUISE_INFO"),
        ("NEW_SIGNAL_3", "CRUISE_INFO"),
        ("BYTE11", "CRUISE_INFO"),
        ("SET_SPEED", "CRUISE_INFO"),
        ("NEW_SIGNAL_4", "CRUISE_INFO"),
      ]

      signals += [(f"BYTE{i}", "CRUISE_INFO") for i in range(3, 7)]
      signals += [(f"BYTE{i}", "CRUISE_INFO") for i in range(13, 31)]

      checks = [
        ("CRUISE_INFO", 50),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 6)
