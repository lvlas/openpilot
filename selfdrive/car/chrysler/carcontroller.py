from numpy import interp
from cereal import car
from openpilot.selfdrive.car.chrysler.chryslercan import create_lkas_hud, create_lkas_command, \
  create_wheel_buttons, create_mango_hud, create_op_acc_1, create_op_acc_2, create_op_dashboard, create_op_chime
from opendbc.can.packer import CANPacker
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_meas_steer_torque_limits
from openpilot.selfdrive.car.interfaces import GearShifter
from openpilot.common.params import Params
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.car.chrysler.chryslerlonghelper import cluster_chime, accel_hysteresis, accel_rate_limit, \
  cruiseiconlogic, setspeedlogic, SET_SPEED_MIN, DEFAULT_DECEL, STOP_GAS_THRESHOLD, START_BRAKE_THRESHOLD, \
  STOP_BRAKE_THRESHOLD, START_GAS_THRESHOLD, CHIME_GAP_TIME, ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.car.chrysler.values import RAM_CARS, CarControllerParams, ChryslerFlags
from openpilot.selfdrive.car.interfaces import CarControllerBase

LongCtrlState = car.CarControl.Actuators.LongControlState

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.ccframe = 0
    self.hud_count = 0
    self.car_fingerprint = CP.carFingerprint
    self.gone_fast_yet = False
    self.steer_rate_limited = False
    self.timer = 0
    self.steerErrorMod = False
    self.steer_type = int(0)
    self.hightorqUnavailable = False
    self.acc_stop_timer = 0
    self.stop_button_spam = 0
    self.wheel_button_counter_prev = 0
    self.lead_dist_at_stop = 0
    self.hybridEcu = CP.enablehybridEcu
    self.mango_lat_active = Params().get_bool('ChryslerMangoLat')
    self.full_range_steer = Params().get_bool('LkasFullRangeAvailable')
    self.mango_mode_active = self.mango_lat_active or self.full_range_steer
    #OPLong starts here
    self.op_long_enable = True #CP.openpilotLongitudinalControl
    self.acc_available = True
    self.acc_enabled = False
    self.set_speed = SET_SPEED_MIN
    self.set_speed_timer = 0
    self.short_press = True
    self.gas_speed_sync = False
    self.cruise_state = 0
    self.cruise_icon = 0
    self.follow_set = 4
    self.follow_set_prev = 4
    self.acc_pre_brake = False
    self.accel_lim_prev = 0.
    self.accel_lim = 0.
    self.accel_steady = 0.
    self.accel_active = False
    self.decel_active = False
    self.chime = 0
    self.chime_timer = 0
    self.gap_timer = 0
    self.enabled_prev = False
    self.resume_set_speed = 0
    self.allow_resume_button = False
    self.acc_counter = 0
    self.gas_timer = 0
    self.go_req = False
    self.stop_req = False
    self.decel_val_prev = 0.
    self.done = False
    self.accel = 0.

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)

  def update(self, CC, CS, now_nanos):

    # *** compute control surfaces ***
    enabled = CC.enabled
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    wp_type = int(0)
    self.hightorqUnavailable = False

    if self.full_range_steer:
      wp_type = int(1)
    if self.mango_lat_active:
      wp_type = int(2)

    if enabled:
      if self.timer < 99 and wp_type == 1 and CS.out.vEgo < 65:
        self.timer += 1
      else:
        self.timer = 99
    else:
      self.timer = 0

    lkas_active = self.timer == 99 and  (self.ccframe >= 500)

    # steer torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)

    if not self.mango_mode_active:
      moving_fast = CS.out.vEgo > self.CP.minSteerSpeed  # for status message
      if CS.out.vEgo > (self.CP.minSteerSpeed - 0.5):  # for command high bit
        self.gone_fast_yet = True
      elif self.car_fingerprint in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
          self.gone_fast_yet = False  # < 14.5m/s stock turns off this bit, but fine down to 13.5
      lkas_active = moving_fast and enabled

      if not lkas_active:
        apply_steer = 0

    self.steer_rate_limited = new_steer != apply_steer
    self.apply_steer_last = apply_steer
    self.steer_type = wp_type

    if wp_type != 2:
      self.steerErrorMod = CS.steerError
      if self.steerErrorMod:
        self.steer_type = int(0)
    elif CS.apaFault or CS.out.gearShifter not in (GearShifter.drive, GearShifter.low) or \
            not CS.veh_on or CS.apa_steer_status:
      self.steer_type = int(0)
    
    if (self.ccframe < 500) or \
            (self.steer_type == int(0) and CS.out.gearShifter in (GearShifter.drive, GearShifter.low) and not CS.apaFault and self.mango_lat_active):
      self.hightorqUnavailable = True

    self.apaActive = CS.apasteerOn and self.steer_type == 2

    can_sends = []

    #*** control msgs ***

    self.resume_press = False
    if CS.acc_hold and CS.out.standstill:
      self.acc_stop_timer += 1
      if self.acc_stop_timer > 100: # send resume spam at 1.8 sec; looks like ACC auto resumes by itself if lead moves within 2 seconds
        self.resume_press = True
    else:
      self.acc_stop_timer = 0
      self.lead_dist_at_stop = CS.lead_dist

    if CS.acc_button_pressed:
      self.stop_button_spam = self.ccframe + 50 # stop spamming for 500msec if driver pressed any acc steering wheel button

    wheel_button_counter_change = CS.wheel_button_counter != self.wheel_button_counter_prev
    if wheel_button_counter_change:
      self.wheel_button_counter_prev = CS.wheel_button_counter

    self.op_cancel_cmd = False

    if (self.ccframe % 8 < 4) and self.ccframe >= self.stop_button_spam:  #and wheel_button_counter_change
      button_type = None
      if not enabled and pcm_cancel_cmd and CS.out.cruiseState.enabled and not self.op_long_enable:
        button_type = 'ACC_CANCEL'
        self.op_cancel_cmd = True
      elif enabled and self.resume_press and not self.op_long_enable and ((CS.lead_dist > self.lead_dist_at_stop) or (CC.hudControl.leadvRel > 0) or (15 > CS.lead_dist >= 6.)):
        button_type = 'ACC_RESUME'
      elif (enabled and CS.out.standstill):
        button_type = 'ACC_RESUME'

      if button_type is not None:
        new_msg = create_wheel_buttons(self.packer, CS.wheel_button_counter + 1, button_type)
        can_sends.append(new_msg)

    # LKAS_HEARTBIT is forwarded by Panda so no need to send it here.
    # frame is 100Hz (0.01s period)
    if (self.ccframe % 2 == 0) and wp_type == 2:  # 0.02s period
      new_msg = create_mango_hud(
          self.packer, self.apaActive, CS.apaFault, lkas_active,
          self.steer_type)
      can_sends.append(new_msg)

    if (self.ccframe % 2 == 0) and wp_type != 2:  # 0.25s period
      new_msg = create_lkas_hud(
          self.packer, CS.out.gearShifter, lkas_active,
          self.hud_count, self.steer_type)
      can_sends.append(new_msg)

    if self.ccframe % 25 == 0:
      self.hud_count += 1

    new_msg = create_lkas_command(self.packer, int(apply_steer), lkas_active, CS.lkas_counter)
    can_sends.append(new_msg)


    #############################################################################
    # Chrysler OP long- Recreate ACC ECU here                                   #
    #############################################################################

    # build ACC enabling logic
    ####################################################################################################################
    if CS.acc_on_button and not CS.acc_on_button_prev:
       self.acc_available = not self.acc_available
       if not self.acc_available:
         self.allow_resume_button = False

    self.acc_enabled_prev = self.acc_enabled

    set_button = CS.acc_resume_button or CS.acc_setminus_button if not self.allow_resume_button else CS.acc_setminus_button
    res_button = CS.acc_resume_button if self.allow_resume_button else False

    if not self.acc_enabled and not CS.out.brakePressed and self.acc_available and \
            (CS.acc_setplus_button or set_button or res_button):
      
      self.acc_enabled = True
      if not self.allow_resume_button:
        self.allow_resume_button = True
    elif  self.acc_enabled and not self.acc_available or CS.acc_cancel_button or pcm_cancel_cmd:
      self.acc_enabled = False


    self.set_speed, self.short_press, self.set_speed_timer, \
    self.gas_speed_sync, self.resume_set_speed, self.gas_timer = setspeedlogic(self.set_speed, self.acc_enabled, self.acc_enabled_prev,
                                                               CS.acc_setplus_button, set_button, res_button,
                                                               self.set_speed_timer, self.resume_set_speed, self.short_press,
                                                               CS.out.vEgoRaw, self.gas_speed_sync, CS.out.gasPressed, self.gas_timer)

    self.cruise_state, self.cruise_icon, self.follow_set, self.follow_set_prev = cruiseiconlogic(self.acc_enabled, self.acc_available, CC.hudControl.leadVisible, CS.acc_followdec_button, CS.acc_followinc_button, self.follow_set, self.follow_set_prev)

    # Build ACC long control signals
    ####################################################################################################################
    # gas and brake
    self.accel_lim_prev = self.accel_lim
    self.decel_val = DEFAULT_DECEL
    self.trq_val = CS.axle_torq_min

    long_stopping = actuators.longControlState == LongCtrlState.stopping

    apply_accel = actuators.accel if enabled else 0.

    accmaxBp = [20, 30, 50]
    if Params().get_bool('ChryslerMadGas'):
      accmaxhyb = [ACCEL_MAX, ACCEL_MAX, ACCEL_MAX]
    else:
      accmaxhyb = [ACCEL_MAX, 1., .5]

    if long_stopping and CS.out.standstill and not CS.out.gasPressed:
      self.stop_req = True
    else:
      self.stop_req = False

    if not self.stop_req and CS.out.standstill and enabled:
      self.go_req = True
    else:
      self.go_req = False

    apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady)
    accel_max_tbl = interp(CS.hybrid_power_meter, accmaxBp, accmaxhyb)

    apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, accel_max_tbl)

    self.accel_lim = apply_accel
    apply_accel = accel_rate_limit(self.accel_lim, self.accel_lim_prev, CS.out.standstill)

    if enabled and not CS.out.gasPressed and not self.go_req and\
            (self.stop_req
             or (apply_accel <= min(CS.axle_torq_min/CV.ACCEL_TO_NM, START_BRAKE_THRESHOLD))
             or (self.decel_active and ((CS.out.brake > 10.) or (CS.hybrid_power_meter < 0.)) and
                 (apply_accel < max((CS.axle_torq_min + 20.)/CV.ACCEL_TO_NM, STOP_BRAKE_THRESHOLD)))):
      self.decel_active = True
      self.decel_val = apply_accel
      if self.decel_val_prev > self.decel_val and not self.done:
        self.decel_val = accel_rate_limit(self.decel_val, self.decel_val_prev, CS.out.standstill)
        self.accel = self.decel_val
      else:
        self.done = True

      self.decel_val_prev = self.decel_val
    else:
      self.decel_active = False
      self.done = False
      self.decel_val_prev = CS.out.aEgo

    if enabled and not CS.out.brakePressed and not (CS.out.standstill and (self.stop_req or self.decel_active)) and\
            (apply_accel >= max(START_GAS_THRESHOLD, (CS.axle_torq_min + 20.)/CV.ACCEL_TO_NM)
             or self.accel_active and not self.decel_active and apply_accel > (CS.axle_torq_min - 20.)/CV.ACCEL_TO_NM):
      self.trq_val = apply_accel * CV.ACCEL_TO_NM
      self.accel = apply_accel

      if CS.axle_torq_max > self.trq_val > CS.axle_torq_min:
        self.accel_active = True
      else:
        self.trq_val = CS.axle_torq_min
        self.accel_active = False
      if not self.hybridEcu:
        self.trq_val /= 15.5  # GEAR_RATO guess for non hybrid?
    else:
      self.accel_active = False

    self.chime_mute = True if (CS.out.brakePressed or CS.acc_cancel_button) else False
    self.chime, self.chime_timer, self.gap_timer = cluster_chime(self.chime, enabled, self.enabled_prev, self.chime_timer, self.gap_timer, self.chime_mute)
    self.enabled_prev = enabled
      # Send ACC msgs on can
    ####################################################################################################################
    if self.op_long_enable and self.ccframe % 2 == 0:
      self.acc_counter %= 0xF
      self.acc_counter += 1
      new_msg = create_op_acc_1(self.packer, self.accel_active, self.trq_val, self.acc_counter)
      can_sends.append(new_msg)
      new_msg = create_op_acc_2(self.packer, self.acc_available, self.acc_enabled, self.stop_req, self.go_req,
                                self.acc_pre_brake, self.decel_val, self.decel_active, self.acc_counter)
      can_sends.append(new_msg)
    if self.op_long_enable and self.ccframe % 6 == 0:
      new_msg = create_op_dashboard(self.packer, self.set_speed, self.cruise_state, self.cruise_icon, CC.hudControl.leadVisible,
                                    CC.hudControl.leadDistance, self.op_long_enable)
      can_sends.append(new_msg)

    new_msg = create_op_chime(self.packer, self.chime, self.chime_timer, self.gap_timer, CHIME_GAP_TIME)
    can_sends.append(new_msg)

    self.ccframe += 1

    new_actuators = CC.actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    return new_actuators, can_sends
