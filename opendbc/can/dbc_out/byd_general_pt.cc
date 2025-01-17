#include "common_dbc.h"

namespace {

const Signal sigs_287[] = {
    {
      .name = "COUNTER",
      .b1 = 32,
      .b2 = 8,
      .bo = 24,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "STEER_ANGLE_2",
      .b1 = 0,
      .b2 = 16,
      .bo = 48,
      .is_signed = true,
      .factor = 0.1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "DRIVER_EPS_TORQUE",
      .b1 = 16,
      .b2 = 8,
      .bo = 40,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_290[] = {
    {
      .name = "WHEELSPEED_FL",
      .b1 = 0,
      .b2 = 16,
      .bo = 48,
      .is_signed = false,
      .factor = 0.1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "WHEELSPEED_FR",
      .b1 = 16,
      .b2 = 16,
      .bo = 32,
      .is_signed = false,
      .factor = 0.1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "WHEELSPEED_BL",
      .b1 = 32,
      .b2 = 16,
      .bo = 16,
      .is_signed = false,
      .factor = 0.1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "WHEELSPEED_BR",
      .b1 = 48,
      .b2 = 16,
      .bo = 0,
      .is_signed = false,
      .factor = 0.1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_307[] = {
    {
      .name = "COUNTER",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "GENERIC_TOGGLE",
      .b1 = 24,
      .b2 = 1,
      .bo = 39,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "LEFT_BLINKER",
      .b1 = 34,
      .b2 = 1,
      .bo = 29,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "RIGHT_BLINKER",
      .b1 = 33,
      .b2 = 1,
      .bo = 30,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_482[] = {
    {
      .name = "COUNTER",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "CHECKSUM",
      .b1 = 56,
      .b2 = 8,
      .bo = 0,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "UNKNOWN",
      .b1 = 0,
      .b2 = 14,
      .bo = 50,
      .is_signed = true,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_X01",
      .b1 = 8,
      .b2 = 2,
      .bo = 54,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_XE",
      .b1 = 16,
      .b2 = 4,
      .bo = 44,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "STEER_REQ_ACTIVE_LOW",
      .b1 = 19,
      .b2 = 1,
      .bo = 44,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "STEER_REQ",
      .b1 = 18,
      .b2 = 1,
      .bo = 45,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_1_2",
      .b1 = 17,
      .b2 = 1,
      .bo = 46,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_1_1",
      .b1 = 16,
      .b2 = 1,
      .bo = 47,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "STEER_ANGLE",
      .b1 = 24,
      .b2 = 16,
      .bo = 24,
      .is_signed = true,
      .factor = 0.1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_FF",
      .b1 = 40,
      .b2 = 8,
      .bo = 16,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_F",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_496[] = {
    {
      .name = "COUNTER",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "CHECKSUM",
      .b1 = 56,
      .b2 = 8,
      .bo = 0,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "WHEELSPEED_CLEAN",
      .b1 = 0,
      .b2 = 16,
      .bo = 48,
      .is_signed = false,
      .factor = 0.1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_508[] = {
    {
      .name = "COUNTER",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "CHECKSUM",
      .b1 = 56,
      .b2 = 8,
      .bo = 0,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "MAIN_TORQUE",
      .b1 = 0,
      .b2 = 16,
      .bo = 48,
      .is_signed = true,
      .factor = 0.1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_544[] = {
    {
      .name = "PEDAL_PRESSED_ACTIVE_LOW",
      .b1 = 11,
      .b2 = 1,
      .bo = 52,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_578[] = {
    {
      .name = "RAW_THROTTLE",
      .b1 = 25,
      .b2 = 7,
      .bo = 32,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "BRAKE_PRESSED",
      .b1 = 34,
      .b2 = 1,
      .bo = 29,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "GEAR",
      .b1 = 40,
      .b2 = 3,
      .bo = 21,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_660[] = {
    {
      .name = "FRONT_RIGHT_DOOR",
      .b1 = 1,
      .b2 = 1,
      .bo = 62,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "FRONT_LEFT_DOOR",
      .b1 = 4,
      .b2 = 1,
      .bo = 59,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "BACK_LEFT_DOOR",
      .b1 = 5,
      .b2 = 1,
      .bo = 58,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "BACK_RIGHT_DOOR",
      .b1 = 0,
      .b2 = 1,
      .bo = 63,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SEATBELT_DRIVER",
      .b1 = 22,
      .b2 = 1,
      .bo = 41,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_790[] = {
    {
      .name = "COUNTER",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "CHECKSUM",
      .b1 = 56,
      .b2 = 8,
      .bo = 0,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "HMA",
      .b1 = 3,
      .b2 = 5,
      .bo = 56,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "STEER_ACTIVE_1_1",
      .b1 = 2,
      .b2 = 1,
      .bo = 61,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "LSS_STATE",
      .b1 = 0,
      .b2 = 2,
      .bo = 62,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "PT3",
      .b1 = 8,
      .b2 = 2,
      .bo = 54,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "HAND_ON_WHEEL_WARNING",
      .b1 = 13,
      .b2 = 1,
      .bo = 50,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "PT2",
      .b1 = 11,
      .b2 = 5,
      .bo = 48,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_XFF",
      .b1 = 16,
      .b2 = 8,
      .bo = 40,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_X5F",
      .b1 = 24,
      .b2 = 8,
      .bo = 32,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_1_2",
      .b1 = 39,
      .b2 = 1,
      .bo = 24,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "PT5",
      .b1 = 33,
      .b2 = 2,
      .bo = 29,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "STEER_ACTIVE_1_3",
      .b1 = 36,
      .b2 = 1,
      .bo = 27,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "STEER_ACTIVE_ACTIVE_LOW",
      .b1 = 35,
      .b2 = 1,
      .bo = 28,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "STEER_ACTIVE_1_2",
      .b1 = 34,
      .b2 = 1,
      .bo = 29,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "PT4",
      .b1 = 32,
      .b2 = 2,
      .bo = 30,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "TSR",
      .b1 = 40,
      .b2 = 8,
      .bo = 16,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SETTINGS",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_813[] = {
    {
      .name = "COUNTER",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "CHECKSUM",
      .b1 = 56,
      .b2 = 8,
      .bo = 0,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_SPEED",
      .b1 = 0,
      .b2 = 8,
      .bo = 56,
      .is_signed = false,
      .factor = 0.5,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_DISTANCE",
      .b1 = 11,
      .b2 = 3,
      .bo = 50,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "ACC_ON2",
      .b1 = 19,
      .b2 = 1,
      .bo = 44,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "ACC_ON1",
      .b1 = 17,
      .b2 = 1,
      .bo = 46,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_XFF",
      .b1 = 40,
      .b2 = 8,
      .bo = 16,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_XF",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_814[] = {
    {
      .name = "COUNTER",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "CHECKSUM",
      .b1 = 56,
      .b2 = 8,
      .bo = 0,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "ACCEL_CMD",
      .b1 = 0,
      .b2 = 8,
      .bo = 56,
      .is_signed = false,
      .factor = 1,
      .offset = -100.0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "ACC_ON_1",
      .b1 = 14,
      .b2 = 1,
      .bo = 49,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_25_1",
      .b1 = 10,
      .b2 = 6,
      .bo = 48,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "ACC_ON_2",
      .b1 = 22,
      .b2 = 1,
      .bo = 41,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_25_2",
      .b1 = 18,
      .b2 = 6,
      .bo = 40,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "DECEL_FACTOR",
      .b1 = 24,
      .b2 = 4,
      .bo = 36,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_X8",
      .b1 = 24,
      .b2 = 4,
      .bo = 36,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "ACCEL_FACTOR",
      .b1 = 32,
      .b2 = 4,
      .bo = 28,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "CMD_REQ_ACTIVE_LOW",
      .b1 = 35,
      .b2 = 1,
      .bo = 28,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_1",
      .b1 = 33,
      .b2 = 1,
      .bo = 30,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "STANDSTILL_RESUME",
      .b1 = 32,
      .b2 = 1,
      .bo = 31,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "STANDSTILL_STATE",
      .b1 = 47,
      .b2 = 1,
      .bo = 16,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "ACC_REQ_NOT_STANDSTILL",
      .b1 = 44,
      .b2 = 1,
      .bo = 19,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "ACC_CONTROLLABLE_AND_ON",
      .b1 = 43,
      .b2 = 1,
      .bo = 20,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "ACC_OVERRIDE_OR_STANDSTILL",
      .b1 = 42,
      .b2 = 1,
      .bo = 21,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_XF",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_834[] = {
    {
      .name = "COUNTER",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "CHECKSUM",
      .b1 = 56,
      .b2 = 8,
      .bo = 0,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "GAS_PEDAL",
      .b1 = 0,
      .b2 = 8,
      .bo = 56,
      .is_signed = false,
      .factor = 0.01,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "BRAKE_PEDAL",
      .b1 = 8,
      .b2 = 8,
      .bo = 48,
      .is_signed = false,
      .factor = 0.01,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_944[] = {
    {
      .name = "COUNTER",
      .b1 = 48,
      .b2 = 4,
      .bo = 12,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "CHECKSUM",
      .b1 = 56,
      .b2 = 8,
      .bo = 0,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_1_1",
      .b1 = 5,
      .b2 = 1,
      .bo = 58,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_BTN",
      .b1 = 4,
      .b2 = 1,
      .bo = 59,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "RES_BTN",
      .b1 = 3,
      .b2 = 1,
      .bo = 60,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "LKAS_ON_BTN",
      .b1 = 1,
      .b2 = 1,
      .bo = 62,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SET_ME_1_2",
      .b1 = 11,
      .b2 = 1,
      .bo = 52,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "DEC_DISTANCE_BTN",
      .b1 = 8,
      .b2 = 1,
      .bo = 55,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "INC_DISTANCE_BTN",
      .b1 = 23,
      .b2 = 1,
      .bo = 40,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "ACC_ON_BTN",
      .b1 = 20,
      .b2 = 1,
      .bo = 43,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_1048[] = {
    {
      .name = "RIGHT_APPROACH",
      .b1 = 22,
      .b2 = 1,
      .bo = 41,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "LEFT_APPROACH",
      .b1 = 16,
      .b2 = 1,
      .bo = 47,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};

const Msg msgs[] = {
  {
    .name = "STEER_MODULE_2",
    .address = 0x11F,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_287),
    .sigs = sigs_287,
  },
  {
    .name = "WHEEL_SPEED",
    .address = 0x122,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_290),
    .sigs = sigs_290,
  },
  {
    .name = "STALKS",
    .address = 0x133,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_307),
    .sigs = sigs_307,
  },
  {
    .name = "STEERING_MODULE_ADAS",
    .address = 0x1E2,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_482),
    .sigs = sigs_482,
  },
  {
    .name = "WHEELSPEED_CLEAN",
    .address = 0x1F0,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_496),
    .sigs = sigs_496,
  },
  {
    .name = "STEERING_TORQUE",
    .address = 0x1FC,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_508),
    .sigs = sigs_508,
  },
  {
    .name = "PEDAL_PRESSED",
    .address = 0x220,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_544),
    .sigs = sigs_544,
  },
  {
    .name = "DRIVE_STATE",
    .address = 0x242,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_578),
    .sigs = sigs_578,
  },
  {
    .name = "METER_CLUSTER",
    .address = 0x294,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_660),
    .sigs = sigs_660,
  },
  {
    .name = "LKAS_HUD_ADAS",
    .address = 0x316,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_790),
    .sigs = sigs_790,
  },
  {
    .name = "ACC_HUD_ADAS",
    .address = 0x32D,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_813),
    .sigs = sigs_813,
  },
  {
    .name = "ACC_CMD",
    .address = 0x32E,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_814),
    .sigs = sigs_814,
  },
  {
    .name = "PEDAL",
    .address = 0x342,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_834),
    .sigs = sigs_834,
  },
  {
    .name = "PCM_BUTTONS",
    .address = 0x3B0,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_944),
    .sigs = sigs_944,
  },
  {
    .name = "BSM",
    .address = 0x418,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_1048),
    .sigs = sigs_1048,
  },
};

const Val vals[] = {
    {
      .name = "GEAR",
      .address = 0x242,
      .def_val = "4 D 2 R 1 P",
      .sigs = sigs_578,
    },
    {
      .name = "SET_DISTANCE",
      .address = 0x32D,
      .def_val = "8 4BAR 4 3BAR 2 2BAR 1 1BAR",
      .sigs = sigs_813,
    },
};

}

const DBC byd_general_pt = {
  .name = "byd_general_pt",
  .num_msgs = ARRAYSIZE(msgs),
  .msgs = msgs,
  .vals = vals,
  .num_vals = ARRAYSIZE(vals),
};

dbc_init(byd_general_pt)