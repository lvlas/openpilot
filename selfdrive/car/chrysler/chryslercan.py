from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.chrysler.values import RAM_CARS

GearShifter = car.CarState.GearShifter
VisualAlert = car.CarControl.HUDControl.VisualAlert

def create_mango_hud(packer, apa_active, apa_fault, enabled, steer_type):
  # LKAS_HUD 0x2a6 (678) Controls what lane-keeping icon is displayed.

  color = 1  # default values are for park or neutral in 2017 are 0 0, but trying 1 1 for 2019
  lines = 1

  # had color = 1 and lines = 1 but trying 2017 hybrid style for now.
  if enabled and apa_active:
      color = 2  # control active, display green.
  if apa_fault:
    color = 3
  values = {
    "LKAS_ICON_COLOR": color,  # byte 0, last 2 bits
    "LKAS_LANE_LINES": lines,  # byte 2, last 4 bits
    "STEER_TYPE": steer_type,
    }
  return packer.make_can_msg("LKAS_HUD", 0, values)  # 0x2a6

def create_lkas_hud(packer, gear, lkas_active, hud_count, steer_type):
  # LKAS_HUD 0x2a6 (678) Controls what lane-keeping icon is displayed.

  color = 1  # default values are for park or neutral in 2017 are 0 0, but trying 1 1 for 2019
  lines = 1
  alerts = 0

  if hud_count < (1 * 4):  # first 3 seconds, 4Hz
    alerts = 1
  # had color = 1 and lines = 1 but trying 2017 hybrid style for now.
  if gear in (GearShifter.drive, GearShifter.reverse, GearShifter.low):
    if lkas_active:
      color = 2  # control active, display green.
      lines = 6
    else:
      color = 1  # control off, display white.
      lines = 1

  values = {
    "LKAS_ICON_COLOR": color,  # byte 0, last 2 bits
    "LKAS_LANE_LINES": lines,  # byte 2, last 4 bits
    "LKAS_ALERTS": alerts,  # byte 3, last 4 bits
    "STEER_TYPE": steer_type,
    }

  return packer.make_can_msg("LKAS_HUD", 0, values)  # 0x2a6


def create_lkas_command(packer, apply_steer, lkas_active, counter):
  # LKAS_COMMAND 0x292 (658) Lane-keeping signal to turn the wheel.
  values = {
    "LKAS_STEERING_TORQUE": apply_steer,
    "LKAS_HIGH_TORQUE": lkas_active,
    "COUNTER": counter,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_wheel_buttons(packer, counter, button_type):
  # WHEEL_BUTTONS (571) Message sent to cancel ACC.
  values = {
    button_type: 1,
    "COUNTER": counter
  }
  return packer.make_can_msg("WHEEL_BUTTONS", 0, values)

def create_op_acc_1(packer, accel_active, trq_val, acc_counter):
  values = { # 20ms
    "ACC_ENG_REQ": accel_active,
    "ACC_TORQ": trq_val,
    "COUNTER": acc_counter
  }
  return packer.make_can_msg("OP_ACC_1", 0, values)

def create_op_acc_2(packer, available, enabled, stop_req, go_req, acc_pre_brake, decel, decel_active, acc_counter):
  values = { # 20ms
    "ACC_STOP": stop_req,
    "ACC_GO": go_req,
    "ACC_DECEL_CMD": decel,
    "ACC_AVAILABLE": available,
    "ACC_ENABLED": enabled,
    "ACC_BRK_PREP": acc_pre_brake,
    "COMMAND_TYPE": 1 if decel_active else 0,
    "COUNTER": acc_counter
  }
  return packer.make_can_msg("OP_ACC_2", 0, values)

def create_op_dashboard(packer, set_speed, cruise_state, cruise_icon, has_lead, lead_d, oplongenable):
  values = { # 60ms
    "ACC_SET_SPEED_KPH": round(set_speed * CV.MS_TO_KPH),
    "ACC_SET_SPEED_MPH": round(set_speed * CV.MS_TO_MPH),
    "CRUISE_STATE": cruise_state,
    "CRUISE_ICON": cruise_icon,
    "LEAD_DIST": min(lead_d, 253) if has_lead else 254,
    "OP_LONG_ENABLE": oplongenable
  }
  return packer.make_can_msg("OP_DASHBOARD", 0, values)

def create_op_chime(packer, chime, chime_timer, gap_timer, chimegap_time):
  values = { # 1000ms
    "CHIME": chime if (chime_timer > 0 and (gap_timer == 0 or gap_timer == chimegap_time)) else 14,
    "CHIME_REQ_L": 1 if (chime_timer > 0 and (gap_timer == 0 or gap_timer == chimegap_time)) else 0,
    "CHIME_REQ_R": 1 if (chime_timer > 0 and (gap_timer == 0 or gap_timer == chimegap_time)) else 0
  }
  return packer.make_can_msg("CHIME", 0, values)

def das_3_command(packer, counter_offset, go, gas, max_gear, stop, brake, brake_prep, das_3):
  values = das_3.copy()  # forward what we parsed
  values['ACC_AVAILABLE'] = 1
  values['ACC_ACTIVE'] = 1
  values['COUNTER'] = (das_3['COUNTER'] + counter_offset) % 0x10

  if go is not None:
    values['ACC_GO'] = go

  if stop is not None:
    values['ACC_STANDSTILL'] = stop

  if brake is not None:
    values['ACC_DECEL_REQ'] = 1
    values['ACC_DECEL'] = brake
    values['ACC_BRK_PREP'] = brake_prep

  if gas is not None:
    values['ENGINE_TORQUE_REQUEST_MAX'] = 1
    values['ENGINE_TORQUE_REQUEST'] = gas

  if max_gear is not None:
    values['GR_MAX_REQ'] = max_gear

  return packer.make_can_msg("DAS_3", 0, values)

def das_5_command(packer, counter_offset, gas, das_5):
  values = das_5.copy()  # forward what we parsed
  values['COUNTER'] = (das_5['COUNTER'] + counter_offset) % 0x10

  if gas is not None:
    values['WHEEL_TORQUE_REQUEST_ACTIVE'] = 1
    values['WHEEL_TORQUE_REQUEST'] = gas

  return packer.make_can_msg("DAS_5", 0, values)

def acc_log(packer, adjustment, aTarget, vTarget, aEgo):
  values = {
    'OP_A_TARGET': aTarget,
    'OP_V_TARGET': vTarget,
    'ADJUSTMENT': adjustment,
    'A_EGO': aEgo
  }
  return packer.make_can_msg("ACC_LOG", 0, values)
