from opendbc.can.packer import CANPacker
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_meas_steer_torque_limits
from openpilot.selfdrive.car.chrysler import chryslercan
from openpilot.selfdrive.car.chrysler.values import RAM_CARS, CarControllerParams, ChryslerFlags
from openpilot.selfdrive.car.interfaces import CarControllerBase, GearShifter
#from openpilot.selfdrive.car.interfaces import GearShifter

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)

    self.ccframe = 0
    self.prev_frame = -1
    self.car_fingerprint = CP.carFingerprint
#    self.gone_fast_yet = False
    self.steer_rate_limited = False
    self.timer = 0
    self.steerErrorMod = False
    self.steer_type = int(0)
    self.on_timer = 0  

  def update(self, CC, CS, now_nanos):
    
    frame = CS.lkas_counter
    if self.prev_frame == frame:
      return []  
    # *** compute control surfaces ***
    if self.on_timer < 200: ########and CS.veh_on:
      self.on_timer += 1
    wp_type = int(2)  
    if CC.enabled:
      if self.timer < 99 and wp_type == 1 and CS.out.vEgo < 65:
        self.timer += 1
      else:
        self.timer = 99
    else:
      self.timer = 0
    lkas_active = self.timer == 99
    new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)
    if not lkas_active: #or not lkas_control_bit:
      apply_steer = 0

    self.steer_rate_limited = new_steer != apply_steer
    self.apply_steer_last = apply_steer
    if CS.out.standstill:
      self.steer_type = wp_type
    if wp_type != 2:
      self.steerErrorMod = CS.steerFaultPermanent #.steerError
      self.steer_type = int(0)
    elif CS.apaFault or CS.out.gearShifter not in (GearShifter.drive, GearShifter.low) or \
            abs(CS.out.steeringAngleDeg) > 330. or self.on_timer < 200 or CS.apa_steer_status:
      self.steer_type = int(0)
    self.apaActive = CS.apasteerOn and self.steer_type == 2
    can_sends = []    
    
    ###can_sends = []

    ###lkas_active = CC.latActive and self.lkas_control_bit_prev

    # cruise buttons
    if (self.frame - self.last_button_frame)*DT_CTRL > 0.05:
      das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0

      # ACC cancellation
      if CC.cruiseControl.cancel:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, cancel=True))

      # ACC resume from standstill
      elif CC.cruiseControl.resume:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, resume=True))

    # HUD alerts
    if self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        new_msg = create_lkas_hud(self.packer, CS.out.gearShifter, self.apaActive, CS.apaFault, hud_alert, lkas_active, self.hud_count, CS.lkas_car_model, self.steer_type)
        can_sends.append(new_msg)
        #can_sends.append(chryslercan.create_lkas_hud(self.packer, self.CP, lkas_active, CC.hudControl.visualAlert,
        #                                             self.hud_count, CS.lkas_car_model, CS.auto_high_beam))
        self.hud_count += 1

    new_msg = create_lkas_command(self.packer, int(apply_steer), lkas_active, frame)
    self.ccframe += 1
    self.prev_frame = frame    

    # steering
#    if self.frame % self.params.STEER_STEP == 0:
#
#      # TODO: can we make this more sane? why is it different for all the cars?
#      lkas_control_bit = self.lkas_control_bit_prev
#      if CS.out.vEgo > self.CP.minSteerSpeed:
#        lkas_control_bit = True
#      elif self.CP.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED:
#        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
#          lkas_control_bit = False
#      elif self.CP.carFingerprint in RAM_CARS:
#        if CS.out.vEgo < (self.CP.minSteerSpeed - 0.5):
#          lkas_control_bit = False
#
#      # EPS faults if LKAS re-enables too quickly
#      lkas_control_bit = lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200)
#
#      if not lkas_control_bit and self.lkas_control_bit_prev:
#        self.last_lkas_falling_edge = self.frame
#      self.lkas_control_bit_prev = lkas_control_bit

      # steer torque
      ###new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
      ###apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)
      ###if not lkas_active or not lkas_control_bit:
      ###  apply_steer = 0
      ###self.apply_steer_last = apply_steer

#      can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_steer), lkas_control_bit))

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    return new_actuators, can_sends
