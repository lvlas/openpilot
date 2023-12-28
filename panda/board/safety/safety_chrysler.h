const LongitudinalLimits CHRYSLER_LONG_LIMITS = {
  // acceleration cmd limits (used for brakes)
  // Signal: ACC_DECEL
  .max_accel = 3685,      // 3685 x 0.004885 - 16 =  2.0 m/s^2
  .min_accel = 2558,      // 2558 x 0.004885 - 16 = -3.5 m/s^2
  .inactive_accel = 4094, // 4094 x 0.004885 - 16 =  4.0 m/s^2

  // gas cmd limits
  // Signal: ENGINE_TORQUE_REQUEST
  .max_gas = 4000,      // 4000 x .25 - 500 =  500.0 Nm
  .min_gas = 0,         //    0 x .25 - 500 = -500.0 Nm
  .inactive_gas = 2000, // 2000 x .25 - 500 =    0.0 Nm
};

enum {
  CHRYSLER_BTN_NONE = 0,
  CHRYSLER_BTN_CANCEL = 1,
  CHRYSLER_BTN_ACCEL = 4,
  CHRYSLER_BTN_DECEL = 8,
  CHRYSLER_BTN_RESUME = 16,
};

bool chrysler_longitudinal = true; //false;

typedef struct {
  const int DAS_4;
  const int DAS_5;
} ChryslerAddrs;

// CAN messages for Chrysler/Jeep platforms
const ChryslerAddrs CHRYSLER_ADDRS = {
  .DAS_3            = 0x1F4,  // ACC state and control from DASM
  .DAS_4            = 0x1F5,  // ACC and FCW dispaly and config from DASM
  .DAS_5            = 0x271,  // ACC and FCW dispaly and config from DASM
};

// CAN messages for the 5th gen RAM DT platform
const ChryslerAddrs CHRYSLER_RAM_DT_ADDRS = {
  .DAS_3            = 0x99,   // ACC state and control from DASM
  .DAS_4            = 0xE8,   // ACC and FCW dispaly and config from DASM
  .DAS_5            = 0xA3,   // ACC and FCW dispaly and config from DASM
};

// CAN messages for the 5th gen RAM HD platform
const ChryslerAddrs CHRYSLER_RAM_HD_ADDRS = {
  .DAS_3            = 0x1F4,  // ACC state and control from DASM
  .DAS_4            = 0x1F5,  // ACC and FCW dispaly and config from DASM
  .DAS_5            = 0x271,  // ACC and FCW dispaly and config from DASM
};

#define CHRYSLER_COMMON_TX_MSGS(addrs, cruise_buttons_bus, lkas_cmd_len)  \
  {(addrs).CRUISE_BUTTONS, (cruise_buttons_bus), 3},                      \
  {(addrs).LKAS_COMMAND, 0, (lkas_cmd_len)},                              \
  {(addrs).DAS_6, 0, 8},                                                  \

#define CHRYSLER_COMMON_LONG_TX_MSGS(addrs)  \
  {(addrs).DAS_3, 0, 8},              \
  {(addrs).DAS_4, 0, 8},              \
  {(addrs).DAS_5, 0, 8},              \

const CanMsg CHRYSLER_LONG_TX_MSGS[] = {
  CHRYSLER_COMMON_TX_MSGS(CHRYSLER_ADDRS, 0, 6)
  CHRYSLER_COMMON_LONG_TX_MSGS(CHRYSLER_ADDRS)
};

const CanMsg CHRYSLER_RAM_DT_TX_MSGS[] = {
  CHRYSLER_COMMON_TX_MSGS(CHRYSLER_RAM_DT_ADDRS, 2, 8)
};

const CanMsg CHRYSLER_RAM_DT_LONG_TX_MSGS[] = {
  CHRYSLER_COMMON_TX_MSGS(CHRYSLER_RAM_DT_ADDRS, 2, 8)
  CHRYSLER_COMMON_LONG_TX_MSGS(CHRYSLER_RAM_DT_ADDRS)
};

const CanMsg CHRYSLER_RAM_HD_TX_MSGS[] = {
  CHRYSLER_COMMON_TX_MSGS(CHRYSLER_RAM_HD_ADDRS, 2, 8)
};

const CanMsg CHRYSLER_RAM_HD_LONG_TX_MSGS[] = {
  CHRYSLER_COMMON_TX_MSGS(CHRYSLER_RAM_HD_ADDRS, 2, 8)
  CHRYSLER_COMMON_LONG_TX_MSGS(CHRYSLER_RAM_HD_ADDRS)
};

#define CHRYSLER_COMMON_RX_CHECKS(addrs)                                                                          \
  {.msg = {{(addrs).EPS_2, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},  \
  {.msg = {{(addrs).ESP_1, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},   \
  {.msg = {{(addrs).ECM_5, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},   \

// TODO: use the same message for both (see vehicle_moving below)
#define CHRYSLER_COMMON_ALT_RX_CHECKS()                                                                 \
  {.msg = {{514, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 100U}, { 0 }, { 0 }}},  \

#define CHRYSLER_COMMON_RAM_RX_CHECKS(addrs)                                                                    \
  {.msg = {{(addrs).ESP_8, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}}, \

#define CHRYSLER_COMMON_ACC_RX_CHECKS(addrs, das_3_bus)                                                                   \
  {.msg = {{(addrs).DAS_3, (das_3_bus), 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}}, \

#define CHRYSLER_COMMON_BUTTONS_RX_CHECKS(addrs)                                                                          \
  {.msg = {{(addrs).CRUISE_BUTTONS, 0, 3, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},  \

RxCheck chrysler_long_rx_checks[] = {
  CHRYSLER_COMMON_RX_CHECKS(CHRYSLER_ADDRS)
  CHRYSLER_COMMON_ALT_RX_CHECKS()
  CHRYSLER_COMMON_BUTTONS_RX_CHECKS(CHRYSLER_ADDRS)
};

RxCheck chrysler_ram_dt_rx_checks[] = {
  CHRYSLER_COMMON_RX_CHECKS(CHRYSLER_RAM_DT_ADDRS)
  CHRYSLER_COMMON_RAM_RX_CHECKS(CHRYSLER_RAM_DT_ADDRS)
  CHRYSLER_COMMON_ACC_RX_CHECKS(CHRYSLER_RAM_DT_ADDRS, 2)
};

RxCheck chrysler_ram_hd_rx_checks[] = {
RxCheck chrysler_ram_dt_long_rx_checks[] = {
  CHRYSLER_COMMON_RX_CHECKS(CHRYSLER_RAM_DT_ADDRS)
  CHRYSLER_COMMON_RAM_RX_CHECKS(CHRYSLER_RAM_DT_ADDRS)
  CHRYSLER_COMMON_BUTTONS_RX_CHECKS(CHRYSLER_RAM_DT_ADDRS)
};

RxCheck chrysler_ram_hd_rx_checks[] = {
  CHRYSLER_COMMON_RX_CHECKS(CHRYSLER_RAM_HD_ADDRS)
  CHRYSLER_COMMON_RAM_RX_CHECKS(CHRYSLER_RAM_HD_ADDRS)
  CHRYSLER_COMMON_ACC_RX_CHECKS(CHRYSLER_RAM_HD_ADDRS, 2)
};

RxCheck chrysler_ram_hd_long_rx_checks[] = {
  CHRYSLER_COMMON_RX_CHECKS(CHRYSLER_RAM_HD_ADDRS)
  CHRYSLER_COMMON_RAM_RX_CHECKS(CHRYSLER_RAM_HD_ADDRS)
  CHRYSLER_COMMON_BUTTONS_RX_CHECKS(CHRYSLER_RAM_HD_ADDRS)
};

const uint32_t CHRYSLER_PARAM_RAM_DT = 1U;  // set for Ram DT platform
const uint32_t CHRYSLER_PARAM_RAM_HD = 2U;  // set for Ram HD platform
const uint32_t CHRYSLER_PARAM_LONGITUDINAL = 4U;

  if (!chrysler_longitudinal) {
    // enter controls on rising edge of ACC, exit controls on ACC off
    const int das_3_bus = (chrysler_platform == CHRYSLER_PACIFICA) ? 0 : 2;
    if ((bus == das_3_bus) && (addr == chrysler_addrs->DAS_3)) {
      bool cruise_engaged = GET_BIT(to_push, 21U) == 1U;
      pcm_cruise_check(cruise_engaged);
    }

  // ACCEL
  if (tx && (addr == chrysler_addrs->DAS_3)) {
    // Signal: ENGINE_TORQUE_REQUEST
    int gas = (((GET_BYTE(to_send, 0) & 0x1FU) << 8) | GET_BYTE(to_send, 1));
    // Signal: ACC_DECEL
    int accel = (((GET_BYTE(to_send, 2) & 0xFU) << 8) | GET_BYTE(to_send, 3));

    bool violation = false;
    violation |= longitudinal_accel_checks(accel, CHRYSLER_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas, CHRYSLER_LONG_LIMITS);

    if (violation) {
      tx = 0;
    }
  }





const int CHRYSLER_MAX_STEER = 261;
const int CHRYSLER_MAX_RT_DELTA = 112;        // max delta torque allowed for real time checks
const uint32_t CHRYSLER_RT_INTERVAL = 250000;  // 250ms between real time checks
const int CHRYSLER_MAX_RATE_UP = 3;
const int CHRYSLER_MAX_RATE_DOWN = 3;
const int CHRYSLER_MAX_TORQUE_ERROR = 320;    // max torque cmd in excess of torque motor
const int CHRYSLER_GAS_THRSLD = 30;  // 7% more than 2m/s
const int CHRYSLER_STANDSTILL_THRSLD = 10;  // about 1m/s
const CanMsg CHRYSLER_TX_MSGS[] = {/*{571, 0, 3}, {658, 0, 6}, {678, 0, 8},
                                   {502, 0, 8}, {503, 0, 8}, {626, 0, 8}, {838, 0, 2}*/   CHRYSLER_COMMON_TX_MSGS(CHRYSLER_ADDRS, 0, 6)};  //OP long msgs to WP asi uz ne

AddrCheckStruct chrysler_addr_checks[] = {
  {.msg = {{544, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  //{.msg = {{514, 0, 8, .check_checksum = false, .max_counter = 0U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{500, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{308, 0, 8, .check_checksum = false, .max_counter = 15U,  .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{320, 0, 8, .check_checksum = true, .max_counter = 15U,  .expected_timestep = 20000U}, { 0 }, { 0 }}},

  {.msg = {{(addrs).EPS_2, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},  
  {.msg = {{(addrs).ESP_1, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},   
  {.msg = {{(addrs).ECM_5, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},  
  {.msg = {{514, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{(addrs).DAS_3, (das_3_bus), 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
};
#define CHRYSLER_ADDR_CHECK_LEN (sizeof(chrysler_addr_checks) / sizeof(chrysler_addr_checks[0]))
addr_checks chrysler_rx_checks = {chrysler_addr_checks, CHRYSLER_ADDR_CHECK_LEN};

static uint32_t chrysler_get_checksum(CANPacket_t *to_push) {
  int checksum_byte = GET_LEN(to_push) - 1U;
  return (uint8_t)(GET_BYTE(to_push, checksum_byte));
}

static uint32_t chrysler_compute_checksum(CANPacket_t *to_push) {
  // TODO: clean this up
  // http://illmatics.com/Remote%20Car%20Hacking.pdf
  uint8_t checksum = 0xFFU;
  int len = GET_LEN(to_push);
  for (int j = 0; j < (len - 1); j++) {
    uint8_t shift = 0x80U;
    uint8_t curr = (uint8_t)GET_BYTE(to_push, j);
    for (int i=0; i<8; i++) {
      uint8_t bit_sum = curr & shift;
      uint8_t temp_chk = checksum & 0x80U;
      if (bit_sum != 0U) {
        bit_sum = 0x1C;
        if (temp_chk != 0U) {
          bit_sum = 1;
        }
        checksum = checksum << 1;
        temp_chk = checksum | 1U;
        bit_sum ^= temp_chk;
      } else {
        if (temp_chk != 0U) {
          bit_sum = 0x1D;
        }
        checksum = checksum << 1;
        bit_sum ^= checksum;
      }
      checksum = bit_sum;
      shift = shift >> 1;
    }
  }
  return (uint8_t)(~checksum);
}

static uint8_t chrysler_get_counter(CANPacket_t *to_push) {
  int counter_byte = GET_LEN(to_push) - 2U;
  return (uint8_t)(GET_BYTE(to_push, counter_byte) >> 4);
}

static int chrysler_rx_hook(CANPacket_t *to_push) {




  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  if ((bus == 0) && (addr == chrysler_addrs->CRUISE_BUTTONS) && chrysler_longitudinal) {
    int cruise_button = GET_BIT(to_push, 0U);       // cancel button
    // ensure cancel overrides any multi-button pressed state
    if (!cruise_button) {
      cruise_button |= GET_BIT(to_push, 2U) << 2U;  // accel button
      cruise_button |= GET_BIT(to_push, 3U) << 3U;  // decel button
      cruise_button |= GET_BIT(to_push, 4U) << 4U;  // resume button
    }

    // enter controls on falling edge of accel/decel/resume
    bool accel = (cruise_button != CHRYSLER_BTN_ACCEL) && (cruise_button_prev == CHRYSLER_BTN_ACCEL);
    bool decel = (cruise_button != CHRYSLER_BTN_DECEL) && (cruise_button_prev == CHRYSLER_BTN_DECEL);
    bool resume = (cruise_button != CHRYSLER_BTN_RESUME) && (cruise_button_prev == CHRYSLER_BTN_RESUME);
    if (accel || decel || resume) {
      controls_allowed = true;
    }

    // exit controls on cancel press
    if (cruise_button == CHRYSLER_BTN_CANCEL) {
      controls_allowed = false;
    }

    cruise_button_prev = cruise_button;




  
  bool unsafe_chrysler_mango = alternative_experience & UNSAFE_CHRYSLER_MANGO;

  bool valid = addr_safety_check(to_push, &chrysler_rx_checks,
                                 chrysler_get_checksum, chrysler_compute_checksum,
                                 chrysler_get_counter, NULL);

  if (valid && (GET_BUS(to_push) == 0U)) {
    int addr = GET_ADDR(to_push);

    // Measured eps torque
    if (addr == 544) {
      int torque_meas_new = ((GET_BYTE(to_push, 4) & 0x7U) << 8) + GET_BYTE(to_push, 5) - 1024U;
      if (unsafe_chrysler_mango) {
        torque_meas_new = torque_meas_new/4;
      }
      // update array of samples
      update_sample(&torque_meas, torque_meas_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (addr == 500) {
      int cruise_engaged = ((GET_BYTE(to_push, 2) & 0x38U) >> 3) == 7U;
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // update speed
    if (addr == 514) {
      int speed_l = (GET_BYTE(to_push, 0) << 4) + (GET_BYTE(to_push, 1) >> 4);
      int speed_r = (GET_BYTE(to_push, 2) << 4) + (GET_BYTE(to_push, 3) >> 4);
      vehicle_moving = (speed_l != 0) || (speed_r != 0);
    }

    // exit controls on rising edge of gas press
    if (addr == 559) {
      gas_pressed = GET_BYTE(to_push, 0U) != 0U;
    }

    // exit controls on rising edge of brake press
    if (addr == 320) {
      brake_pressed = (GET_BYTE(to_push, 0) & 0x7U) == 5U;
      if (brake_pressed && (!brake_pressed_prev || vehicle_moving)) {
        controls_allowed = 0;
      }
      brake_pressed_prev = brake_pressed;
    }

    generic_rx_checks((addr == 0x292));
  }
  return valid;
}

uint32_t ts_last = 0;

static int chrysler_tx_hook(CANPacket_t *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);

  if (!msg_allowed(to_send, CHRYSLER_TX_MSGS, sizeof(CHRYSLER_TX_MSGS) / sizeof(CHRYSLER_TX_MSGS[0]))) {
    tx = 0;
  }

  // LKA STEER
  if (addr == 0x292) {
    int desired_torque = ((GET_BYTE(to_send, 0) & 0x7U) << 8) + GET_BYTE(to_send, 1) - 1024U;
    uint32_t ts = microsecond_timer_get();
    bool violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, CHRYSLER_MAX_STEER, -CHRYSLER_MAX_STEER);

      // *** torque rate limit check ***
      violation |= dist_to_meas_check(desired_torque, desired_torque_last,
        &torque_meas, CHRYSLER_MAX_RATE_UP, CHRYSLER_MAX_RATE_DOWN, CHRYSLER_MAX_TORQUE_ERROR);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, CHRYSLER_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > CHRYSLER_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) {
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  else if ((addr == 571) && !chrysler_longitudinal) {
    if (((GET_BYTE(to_send, 0) != 1U) && (GET_BYTE(to_send, 0) != 16U)) // ACC_CANCEL && ACC_RESUME
        || ((GET_BYTE(to_send, 1) & 1) == 1U)) {
      tx = 0;
    }
  }

  return tx;
}

static int chrysler_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  // forward CAN 0 -> 2 so stock LKAS camera sees messages
  const bool is_buttons = (addr == chrysler_addrs->CRUISE_BUTTONS);
  if ((bus_num == 0) && (!chrysler_longitudinal || !is_buttons)) {6
    bus_fwd = 2;
  }

  // forward all messages from camera except LKAS_COMMAND and LKAS_HUD
  if ((bus_num == 2) && (addr != 658) && (addr != 678)) {
    bus_fwd = 0;
  }

  const bool is_lkas = ((addr == chrysler_addrs->LKAS_COMMAND) || (addr == chrysler_addrs->DAS_6));
  const bool is_acc = ((addr == chrysler_addrs->DAS_3) || (addr == chrysler_addrs->DAS_4) || (addr == chrysler_addrs->DAS_5));
  if ((bus_num == 2) && !is_lkas && (!chrysler_longitudinal || !is_acc)){
    bus_fwd = 0;
  }

  return bus_fwd;
}

static const addr_checks* chrysler_init(uint16_t param) {
  UNUSED(param);
  return &chrysler_rx_checks;
}

const safety_hooks chrysler_hooks = {
  .init = chrysler_init,
  .rx = chrysler_rx_hook,
  .tx = chrysler_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = chrysler_fwd_hook,
};
