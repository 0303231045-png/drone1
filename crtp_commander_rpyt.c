#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "crtp_commander.h"
#include "commander.h"
#include "estimator.h"
#include "crtp.h"
#include "param.h"
#include "FreeRTOS.h"
#include "num.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "MODE"
#include "debug_cf.h"

#define MIN_THRUST  1000
#define MAX_THRUST  60000

typedef enum { RATE = 0, ANGLE = 1 } RPYType;
typedef enum { CAREFREE = 0, PLUSMODE = 1, XMODE = 2 } YawModeType;

static RPYType stabilizationModeRoll  = ANGLE;
static RPYType stabilizationModePitch = ANGLE;
static RPYType stabilizationModeYaw   = RATE;
static YawModeType yawMode = DEFAULT_YAW_MODE;
static bool carefreeResetFront;

static bool thrustLocked = true;
static bool altHoldMode = false;
static bool posHoldMode = false;
static bool posSetMode = false;

void setCommandermode(FlightMode mode){
#ifdef CONFIG_ENABLE_COMMAND_MODE_SET
  switch (mode) {
  case ALTHOLD_MODE: altHoldMode = true; posHoldMode = false; posSetMode = false; registerRequiredEstimator(complementaryEstimator); break;
  case POSHOLD_MODE: altHoldMode = true; posHoldMode = true; posSetMode = false; registerRequiredEstimator(kalmanEstimator); break;
  case POSSET_MODE: altHoldMode = false; posHoldMode = false; posSetMode = true; registerRequiredEstimator(kalmanEstimator); break;
  default: altHoldMode = false; posHoldMode = false; posSetMode = false; registerRequiredEstimator(complementaryEstimator); break;
  }
#endif
}

static void rotateYaw(setpoint_t *setpoint, float yawRad) {
  float cosy = cosf(yawRad);
  float siny = sinf(yawRad);
  float originalRoll = setpoint->attitude.roll;
  float originalPitch = setpoint->attitude.pitch;
  setpoint->attitude.roll = originalRoll * cosy - originalPitch * siny;
  setpoint->attitude.pitch = originalPitch * cosy + originalRoll * siny;
}

static void yawModeUpdate(setpoint_t *setpoint) {
  switch (yawMode) {
    case PLUSMODE: rotateYaw(setpoint, 45 * M_PI / 180); break;
    case XMODE: default: break;
  }
}

// --- HÀM GIẢI MÃ ---
void crtpCommanderRpytDecodeSetpoint(setpoint_t *setpoint, CRTPPacket *pk)
{
  uint8_t *raw = pk->data;

  float inRoll, inPitch, inYaw;
  uint16_t inThrust;

  memcpy(&inRoll,  &raw[0],  4);
  memcpy(&inPitch, &raw[4],  4);
  memcpy(&inYaw,   &raw[8],  4);
  inThrust = raw[12] | (raw[13] << 8);

  /* ========== THRUST ========== */
  if (inThrust < MIN_THRUST) {
    setpoint->thrust = 0;
  } else {
    setpoint->thrust = fminf(inThrust, MAX_THRUST);
  }

  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  setpoint->mode.z = modeDisable;

  /* ========== ROLL ========== */
  if (stabilizationModeRoll == RATE) {
    setpoint->mode.roll = modeVelocity;
    setpoint->attitudeRate.roll = inRoll;
    setpoint->attitude.roll = 0;
  } else {
    setpoint->mode.roll = modeAbs;
    setpoint->attitude.roll = inRoll;
    setpoint->attitudeRate.roll = 0;
  }

  /* ========== PITCH ========== */
  if (stabilizationModePitch == RATE) {
    setpoint->mode.pitch = modeVelocity;
    setpoint->attitudeRate.pitch = inPitch;
    setpoint->attitude.pitch = 0;
  } else {
    setpoint->mode.pitch = modeAbs;
    setpoint->attitude.pitch = inPitch;
    setpoint->attitudeRate.pitch = 0;
  }

  /* ========== YAW ========== */
if (stabilizationModeYaw == RATE) {
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = -inYaw;
  setpoint->attitude.yaw = 0;
} else {
  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = inYaw;
  setpoint->attitudeRate.yaw = 0;
}
}

PARAM_GROUP_START(flightmode)
PARAM_ADD(PARAM_UINT8, althold, &altHoldMode)
PARAM_ADD(PARAM_UINT8, poshold, &posHoldMode)
PARAM_ADD(PARAM_UINT8, posSet, &posSetMode)
PARAM_ADD(PARAM_UINT8, yawMode, &yawMode)
PARAM_ADD(PARAM_UINT8, yawRst, &carefreeResetFront)
PARAM_ADD(PARAM_UINT8, stabModeRoll, &stabilizationModeRoll)
PARAM_ADD(PARAM_UINT8, stabModePitch, &stabilizationModePitch)
PARAM_ADD(PARAM_UINT8, stabModeYaw, &stabilizationModeYaw)
PARAM_GROUP_STOP(flightmode)




