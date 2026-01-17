/**
 * Crazyflie control firmware
 *
 * power_distribution_stock.c
 * Stock power distribution logic (Quad X / Normal)
 * Rewritten with Safety Checks
 */

#include <string.h>
#include <stdbool.h>

#include "power_distribution.h"
#include "motors.h"
#include "platform.h"
#include "log.h"
#include "param.h"
#include "num.h"

#define DEBUG_MODULE "PWR_DIST"
#include "debug_cf.h"

/* =========================================================
 * CẤU HÌNH (CONFIG)
 * ========================================================= */

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 3000
#endif

/* =========================================================
 * BIẾN NỘI BỘ (INTERNAL VARIABLES)
 * ========================================================= */

static bool motorSetEnable = false;
static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

/* Biến lưu giá trị thực tế gửi xuống motor (để Log) */
static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

/* Biến dùng cho chế độ Test Motor thủ công (qua Param) */
static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

/* =========================================================
 * INIT
 * ========================================================= */

void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
  return motorsTest();
}

/* =========================================================
 * UTILITY
 * ========================================================= */

/* Macro giới hạn giá trị trong khoảng uint16 (0 - 65535) */
#define limitThrust(x) limitUint16(x)

void powerStop(void)
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

/* =========================================================
 * CORE DISTRIBUTION (TRỘN XUNG)
 * ========================================================= */

void powerDistribution(const control_t *control)
{
  int32_t m1 = 0;
  int32_t m2 = 0;
  int32_t m3 = 0;
  int32_t m4 = 0;

  /* ---------------------------------------------------------
   * BƯỚC 1: TÍNH TOÁN MIXING (TRỘN ROLL/PITCH/YAW)
   * Chỉ tính toán nếu có lực đẩy (Thrust > 0)
   * --------------------------------------------------------- */
  if (control->thrust > 0)
  {
#ifdef QUAD_FORMATION_X
    /* Chia đôi Roll/Pitch để tránh bão hòa động cơ quá sớm */
    int16_t r = control->roll  >> 1;
    int16_t p = control->pitch >> 1;

    // Front Left (CCW)
    m1 = control->thrust + r + p - control->yaw;

    // Front Right (CW)
    m2 = control->thrust - r + p + control->yaw;

    // Back Right (CCW)
    m3 = control->thrust - r - p - control->yaw;

    // Back Left (CW)
    m4 = control->thrust + r - p + control->yaw;
#else
    // Cấu hình dấu cộng (+)
    m1 = control->thrust + control->pitch + control->yaw;
    m2 = control->thrust - control->roll  - control->yaw;
    m3 = control->thrust - control->pitch + control->yaw;
    m4 = control->thrust + control->roll  - control->yaw;
#endif

    /* ---------------------------------------------------------
     * BƯỚC 2: IDLE THRUST (GA CẦM CHỪNG)
     * Chỉ áp dụng khi đang bay (Thrust > 0) để giữ motor quay đều
     * --------------------------------------------------------- */
    if (m1 < idleThrust) m1 = idleThrust;
    if (m2 < idleThrust) m2 = idleThrust;
    if (m3 < idleThrust) m3 = idleThrust;
    if (m4 < idleThrust) m4 = idleThrust;
  }
  else
  {
    /* ---------------------------------------------------------
     * AN TOÀN: NẾU THRUST = 0 -> TẮT HẲN MOTOR
     * --------------------------------------------------------- */
    m1 = 0;
    m2 = 0;
    m3 = 0;
    m4 = 0;
  }

  /* ---------------------------------------------------------
   * BƯỚC 3: GIỚI HẠN & CẬP NHẬT BIẾN LOG
   * --------------------------------------------------------- */
  motorPower.m1 = limitThrust(m1);
  motorPower.m2 = limitThrust(m2);
  motorPower.m3 = limitThrust(m3);
  motorPower.m4 = limitThrust(m4);

  /* ---------------------------------------------------------
   * BƯỚC 4: CHẾ ĐỘ TEST MOTOR (GHI ĐÈ NẾU BẬT)
   * --------------------------------------------------------- */
  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
    return;
  }

  /* ---------------------------------------------------------
   * BƯỚC 5: GỬI LỆNH XUỐNG ESC
   * --------------------------------------------------------- */
  motorsSetRatio(MOTOR_M1, motorPower.m1);
  motorsSetRatio(MOTOR_M2, motorPower.m2);
  motorsSetRatio(MOTOR_M3, motorPower.m3);
  motorsSetRatio(MOTOR_M4, motorPower.m4);
}

/* =========================================================
 * PARAM (CẤU HÌNH TỪ XA)
 * ========================================================= */

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8,  enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1,     &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2,     &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3,     &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4,     &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

PARAM_GROUP_START(powerDist)
PARAM_ADD(PARAM_UINT32, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

/* =========================================================
 * LOG (GHI NHẬT KÝ)
 * ========================================================= */

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)