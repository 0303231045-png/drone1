#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "commander.h"
#include "crtp_commander.h"
#include "crtp_commander_high_level.h"

#include "cf_math.h"
#include "param.h"
#include "stm32_legacy.h"
#include "static_mem.h"

// --- THÊM THƯ VIỆN LOG ---
#define DEBUG_MODULE "COMMANDER"
#include "debug_cf.h" 
// -------------------------

static bool isInit;
const static setpoint_t nullSetpoint;
static setpoint_t tempSetpoint;
static state_t lastState;
const static int priorityDisable = COMMANDER_PRIORITY_DISABLE;

static uint32_t lastUpdate;
static bool enableHighLevel = false;

static QueueHandle_t setpointQueue;
STATIC_MEM_QUEUE_ALLOC(setpointQueue, 1, sizeof(setpoint_t));
static QueueHandle_t priorityQueue;
STATIC_MEM_QUEUE_ALLOC(priorityQueue, 1, sizeof(int));

// --- BIẾN CHO TÍNH NĂNG ARMING ---
static bool isArmed = false;
static uint32_t armHoldStart = 0;
// Ngưỡng gạt Yaw để Arm/Disarm (Đơn vị độ/giây hoặc raw rate)
   #define THRUST_ARM_MAX 0.05f
#define YAW_ARM_THRESHOLD    -40.0f  // Gạt trái
#define YAW_DISARM_THRESHOLD  40.0f  // Gạt phải
// ----------------------------------

/* Public functions */
void commanderInit(void)
{
  setpointQueue = STATIC_MEM_QUEUE_CREATE(setpointQueue);
  ASSERT(setpointQueue);
  xQueueSend(setpointQueue, &nullSetpoint, 0);

  priorityQueue = STATIC_MEM_QUEUE_CREATE(priorityQueue);
  ASSERT(priorityQueue);
  xQueueSend(priorityQueue, &priorityDisable, 0);

  crtpCommanderInit();
  crtpCommanderHighLevelInit();
  lastUpdate = xTaskGetTickCount();

  isInit = true;
  isArmed = false; // Mặc định khóa khi khởi động
}

void commanderSetSetpoint(setpoint_t *setpoint, int priority)
{
  int currentPriority;

  const BaseType_t peekResult = xQueuePeek(priorityQueue, &currentPriority, 0);
  ASSERT(peekResult == pdTRUE);

  if (priority >= currentPriority) {
    setpoint->timestamp = xTaskGetTickCount();
    xQueueOverwrite(setpointQueue, setpoint);
    xQueueOverwrite(priorityQueue, &priority);
    crtpCommanderHighLevelStop();
  }
}

void commanderNotifySetpointsStop(int remainValidMillisecs)
{
  uint32_t currentTime = xTaskGetTickCount();
  int timeSetback = MIN(
    COMMANDER_WDT_TIMEOUT_SHUTDOWN - M2T(remainValidMillisecs),
    currentTime
  );
  xQueuePeek(setpointQueue, &tempSetpoint, 0);
  tempSetpoint.timestamp = currentTime - timeSetback;
  xQueueOverwrite(setpointQueue, &tempSetpoint);
  crtpCommanderHighLevelTellState(&lastState);
}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  xQueuePeek(setpointQueue, setpoint, 0);
  lastUpdate = setpoint->timestamp;
  uint32_t currentTime = xTaskGetTickCount();

  /* ================= DEBUG ================= */
  static uint32_t lastDbg = 0;
  if (currentTime - lastDbg > M2T(1000)) {
    lastDbg = currentTime;
    DEBUG_PRINT("[SP] thr=%.3f roll=%.1f pitch=%.1f yaw=%.1f ARM=%d\n",
      setpoint->thrust,
      setpoint->attitude.roll,
      setpoint->attitude.pitch,
      setpoint->attitudeRate.yaw,
      isArmed);
  }
  /* ========================================= */

  /* ============ WATCHDOG ============ */
  if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {

    /* FAILSAFE: mất link -> DISARM */
    isArmed = false;
    armHoldStart = 0;

    if (enableHighLevel) {
      crtpCommanderHighLevelGetSetpoint(setpoint, state);
    }

    if (!enableHighLevel || crtpCommanderHighLevelIsStopped()) {
      memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
    }
  }
  else if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_STABILIZE) {

    xQueueOverwrite(priorityQueue, &priorityDisable);

    /* Leveling nhẹ */
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;

    setpoint->mode.roll  = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->mode.yaw   = modeVelocity;

    setpoint->attitude.roll = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitudeRate.yaw = 0;
  }
  /* ================================== */

  /* ============ ARM / DISARM ============ */
  bool setpointValid =
    (currentTime - setpoint->timestamp) < COMMANDER_WDT_TIMEOUT_STABILIZE;

  /* ---------- ARM (Yaw trái) ---------- */
  if (!isArmed &&
      setpointValid &&
      setpoint->thrust < THRUST_ARM_MAX &&
      setpoint->attitudeRate.yaw < YAW_ARM_THRESHOLD) {

    if (armHoldStart == 0) {
      armHoldStart = currentTime;
    }

    if (currentTime - armHoldStart > M2T(1500)) {
      isArmed = true;
      armHoldStart = 0;
      DEBUG_PRINT("!!! MOTOR UNLOCKED !!!\n");
    }
  }
  else if (!isArmed) {
    armHoldStart = 0;
  }

  /* ---------- DISARM (Yaw phải) ---------- */
  if (isArmed &&
      setpointValid &&
      setpoint->thrust < THRUST_ARM_MAX &&
      setpoint->attitudeRate.yaw > YAW_DISARM_THRESHOLD) {

    isArmed = false;
    armHoldStart = 0;
    DEBUG_PRINT("!!! MOTOR LOCKED !!!\n");
  }
  /* ====================================== */

  /* ============ LOCK OUTPUT WHEN DISARMED ============ */
  if (!isArmed) {

    setpoint->thrust = 0;

    /* Disable toàn bộ control */
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.z = modeDisable;

    setpoint->mode.roll  = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw   = modeDisable;

    setpoint->attitude.roll = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitudeRate.yaw = 0;
  }
  /* =================================================== */

  lastState = *state;
}

bool commanderTest(void)
{
  return isInit;
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

int commanderGetActivePriority(void)
{
  int priority;

  const BaseType_t peekResult = xQueuePeek(priorityQueue, &priority, 0);
  ASSERT(peekResult == pdTRUE);

  return priority;
}

PARAM_GROUP_START(commander)
PARAM_ADD(PARAM_UINT8, enHighLevel, &enableHighLevel)
PARAM_GROUP_STOP(commander)