#!/usr/bin/env python3
from cereal import car
from panda import Panda
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.chrysler.values import CAR, RAM_HD, RAM_DT, RAM_CARS, ChryslerFlags
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.common.params import Params
ButtonType = car.CarState.ButtonEvent.Type

ButtonType = car.CarState.ButtonEvent.Type


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "chrysler"
    ret.dashcamOnly = candidate in RAM_HD

    # radar parsing needs some work, see https://github.com/commaai/openpilot/issues/26842
    ret.radarUnavailable = True # DBC[candidate]['radar'] is None
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4

    # safety config
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.chrysler)]
    if candidate in RAM_HD:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_HD
    elif candidate in RAM_DT:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_DT

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.2)
    ret.minSteerSpeed = 3.8  # m/s

    #ret.lateralTuning.pid.kpBP = [0., 10., 35.]
#    ret.lateralTuning.pid.kpV = [0.02, 0.02, 0.02]

#    ret.lateralTuning.pid.kiBP = [0., 15., 30.]
#    ret.lateralTuning.pid.kiV = [0.003, 0.003, 0.004]

#    ret.lateralTuning.pid.kf = 0.00002   # full torque for 10 deg at 80mph means 0.00007818594

    ret.experimentalLongitudinalAvailable = True #Params().get_bool('ChryslerMangoLong')
    ret.openpilotLongitudinalControl = True #Params().get_bool('ChryslerMangoLong')

    # Long tuning Params -  make individual params for cars, baseline Pacifica Hybrid
    ret.longitudinalTuning.kpBP = [0., 6., 10., 35.]
    ret.longitudinalTuning.kpV = [.4, .6, 0.5, .2]
    ret.longitudinalTuning.kiBP = [0., 30.]
    ret.longitudinalTuning.kiV = [.001, .001]
    ret.stoppingControl = True
    ret.stoppingDecelRate = 0.2

    # Chrysler
    if candidate in (CAR.CHRYSLER_PACIFICA_2017_HYBRID, CAR.CHRYSLER_PACIFICA_2018, CAR.CHRYSLER_PACIFICA_2018_HYBRID, CAR.CHRYSLER_PACIFICA_2019_HYBRID, CAR.CHRYSLER_PACIFICA_2020):
      ret.minSteerSpeed = 0.0 #17.5  if not Params().get_bool('ChryslerMangoLat') and not Params().get_bool('LkasFullRangeAvailable') else 0 # m/s 17 on the way up, 13 on the way down once engaged.
      ret.steerActuatorDelay = 0.2

    # Jeep
    elif candidate in (CAR.JEEP_GRAND_CHEROKEE, CAR.JEEP_GRAND_CHEROKEE_2019):
      #ret.steerActuatorDelay = 0.2
#
      #ret.lateralTuning.init('pid')
      #ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
      #ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]]
      #ret.lateralTuning.pid.kf = 0.00006

      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kiBP = [0.0]
      ret.lateralTuning.pid.kpBP = [0.0]
      ret.lateralTuning.pid.kpV = [0.005]
      ret.lateralTuning.pid.kiV = [0.0005]
      ret.lateralTuning.pid.kf = 0.00004      

    # Ram
    elif candidate == CAR.RAM_1500_5TH_GEN:
      ret.steerActuatorDelay = 0.2
      ret.wheelbase = 3.88
      # Older EPS FW allow steer to zero
      if any(fw.ecu == 'eps' and b"68" < fw.fwVersion[:4] <= b"6831" for fw in car_fw):
        ret.minSteerSpeed = 0.

    elif candidate == CAR.RAM_HD_5TH_GEN:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, 1.0, False)

    else:
      raise ValueError(f"Unsupported car: {candidate}")

    ret.centerToFront = ret.wheelbase * 0.44
    ret.enableBsm = 720 in fingerprint[0]
    ret.enablehybridEcu = 655 in fingerprint[0] or 291 in fingerprint[0]

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)


    ret.steerFaultPermanent = self.CC.steerErrorMod
    ret.hightorqUnavailable = self.CC.hightorqUnavailable

    # events
    events = self.create_common_events(ret, extra_gears=[car.CarState.GearShifter.low])


    if 0: #ret.vEgo < self.CP.minSteerSpeed and not Params().get_bool('ChryslerMangoLat') and not Params().get_bool('LkasFullRangeAvailable'):
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    if self.CC.acc_enabled and (self.CS.accbrakeFaulted or self.CS.accengFaulted):
      events.add(car.CarEvent.EventName.accFaulted)

    ret.events = events.to_msg()

    return ret
