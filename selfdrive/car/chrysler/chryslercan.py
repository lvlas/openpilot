from cereal import car
from openpilot.selfdrive.car.chrysler.values import RAM_CARS

GearShifter = car.CarState.GearShifter
VisualAlert = car.CarControl.HUDControl.VisualAlert

#def create_lkas_hud(packer, CP, lkas_active, hud_alert, hud_count, car_model, auto_high_beam):
  # LKAS_HUD - Controls what lane-keeping icon is displayed

  # == Color ==
  # 0 hidden?
  # 1 white
  # 2 green
  # 3 ldw

  # == Lines ==
  # 03 white Lines
  # 04 grey lines
  # 09 left lane close
  # 0A right lane close
  # 0B left Lane very close
  # 0C right Lane very close
  # 0D left cross cross
  # 0E right lane cross

  # == Alerts ==
  # 7 Normal
  # 6 lane departure place hands on wheel

#  color = 2 if lkas_active else 1
#  lines = 3 if lkas_active else 0
#  alerts = 7 if lkas_active else 0

#  if hud_count < (1 * 4):  # first 3 seconds, 4Hz
#    alerts = 1

#  if hud_alert in (VisualAlert.ldw, VisualAlert.steerRequired):
#    color = 4
#    lines = 0
#    alerts = 6

#  values = {
#    "LKAS_ICON_COLOR": color,
#    "CAR_MODEL": car_model,
#    "LKAS_LANE_LINES": lines,
#    "LKAS_ALERTS": alerts,
#  }

#  if CP.carFingerprint in RAM_CARS:
#    values['AUTO_HIGH_BEAM_ON'] = auto_high_beam

#  return packer.make_can_msg("DAS_6", 0, values)




def create_lkas_hud(packer, gear, apa_active, apa_fault, enabled, hud_count, lkas_car_model, steer_type):
  # LKAS_HUD 0x2a6 (678) Controls what lane-keeping icon is displayed.

  #if hud_alert == VisualAlert.steerRequired:
  #  msg = b'\x00\x00\x00\x03\x00\x00\x00\x00'
  #  return make_can_msg(0x2a6, msg, 0)

  color = 1  # default values are for park or neutral in 2017 are 0 0, but trying 1 1 for 2019
  lines = 1
  alerts = 0

  if hud_count < (1 * 4):  # first 3 seconds, 4Hz
#   alerts = 1
    alerts = 0
  # CAR.PACIFICA_2018_HYBRID and CAR.PACIFICA_2019_HYBRID
  # had color = 1 and lines = 1 but trying 2017 hybrid style for now.
  if gear in (GearShifter.drive, GearShifter.reverse, GearShifter.low):
#    if lkas_active:
#      color = 2  # control active, display green.
#      lines = 6
#    else:
#      color = 1  # control off, display white.
#      lines = 1
    if enabled and apa_active:
      color = 2  # control active, display green.
      lines = 6
  if apa_fault:
    color = 3
  values = {
    "LKAS_ICON_COLOR": color,  # byte 0, last 2 bits
    "CAR_MODEL": lkas_car_model,  # byte 1
    "LKAS_LANE_LINES": lines,  # byte 2, last 4 bits
    "LKAS_ALERTS": alerts,  # byte 3, last 4 bits
    "STEER_TYPE": steer_type,
    }

  return packer.make_can_msg("LKAS_HUD", 0, values)  # 0x2a6





#def create_lkas_command(packer, CP, apply_steer, lkas_control_bit):
#  # LKAS_COMMAND Lane-keeping signal to turn the wheel
#  enabled_val = 2 if CP.carFingerprint in RAM_CARS else 1
#  values = {
#    "STEERING_TORQUE": apply_steer,
#    "LKAS_CONTROL_BIT": enabled_val if lkas_control_bit else 0,
#  }
#  return packer.make_can_msg("LKAS_COMMAND", 0, values)

def create_lkas_command(packer, apply_steer, lkas_active, frame):
  # LKAS_COMMAND 0x292 (658) Lane-keeping signal to turn the wheel.
  values = {
    "LKAS_STEERING_TORQUE": apply_steer,
#    "LKAS_HIGH_TORQUE": int(moving_fast),
    #"LKAS_HIGH_TORQUE": lkas_active,
    "LKAS_CONTROL_BIT": lkas_active,
    "COUNTER": frame % 0x10,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_cruise_buttons(packer, frame, bus, cancel=False, resume=False):
  values = {
    "ACC_Cancel": cancel,
    "ACC_Resume": resume,
    "COUNTER": frame % 0x10,
  }
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)
