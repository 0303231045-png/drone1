#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/mcpwm.h"
#include "esp_log.h"

#include "motors.h"      // Quan trọng: Include header của bạn
#include "platform.h"
#include "log.h"

 #define DEBUG_MODULE "MOTORS"
 #include "debug_cf.h" // Bỏ comment nếu project có file này
const uint16_t testsound[NBR_OF_MOTORS] = { A4, C5, E5, G5 };

// --- CẤU HÌNH ESC ---
// 400Hz là chuẩn cho Drone/ESC hiện đại để bay mượt.
// Nếu ESC quá cũ (loại cho máy bay cánh bằng) thì sửa thành 50.
#define ESC_PWM_FREQ_HZ   400 
#define ESC_MIN_US        1000
#define ESC_MAX_US        2000

// --- BIẾN TOÀN CỤC ---
static bool isInit = false;
static uint16_t motor_us[NBR_OF_MOTORS] = {1000, 1000, 1000, 1000};


static const MotorPerifDef **motorMap;

// --- GPIO MAP ---
/* * MCPWM Unit 0
 * Timer 0: Dùng cho Motor 1 (Op A) & Motor 2 (Op B)
 * Timer 1: Dùng cho Motor 3 (Op A) & Motor 4 (Op B)
 */
static const mcpwm_io_signals_t mcpwm_signal[NBR_OF_MOTORS] = {
    MCPWM0A, // M1 - Timer 0
    MCPWM0B, // M2 - Timer 0
    MCPWM1A, // M3 - Timer 1
    MCPWM1B  // M4 - Timer 1
};

static const int motor_gpio[NBR_OF_MOTORS] = {
    MOTOR1_GPIO,
    MOTOR2_GPIO,
    MOTOR3_GPIO,
    MOTOR4_GPIO
};

/* ===== HÀM INIT CƠ BẢN (Theo header) ===== */
bool pwm_timmer_init(void)
{
    // Với ESP32 MCPWM, ta init trong motorsInit luôn.
    // Hàm này giữ lại để tương thích với cấu trúc cũ.
    return true; 
}

/* ===== INIT CHÍNH ===== */
void motorsInit(const MotorPerifDef **motorMapSelect)
{
    if (isInit) return;

    motorMap = motorMapSelect;

    mcpwm_config_t pwm_cfg = {
        .frequency = ESC_PWM_FREQ_HZ,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };

    // 1. Khởi tạo Timer 0 (Cho Motor 1, 2)
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_cfg);
    
    // 2. Khởi tạo Timer 1 (Cho Motor 3, 4) - FIX LỖI GIẬT ĐỘNG CƠ
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_cfg);

    // 3. Gán chân GPIO
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        // Kiểm tra xem GPIO có hợp lệ không (tránh lỗi config -1)
        if(motor_gpio[i] >= 0) {
            mcpwm_gpio_init(MCPWM_UNIT_0, mcpwm_signal[i], motor_gpio[i]);
        }
    }

    // ARM ESC: Gửi xung 1000us để ESC nhận điểm 0
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetRatio(i, 0);
    }

    // Chờ ESC khởi động (quan trọng)
    vTaskDelay(pdMS_TO_TICKS(2000));

    isInit = true;
    // DEBUG_PRINT("Motors Init: 400Hz Mode\n");
}

void motorsDeInit(const MotorPerifDef **motorMapSelect)
{
    (void)motorMapSelect;
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetRatio(i, 0);
    }
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
    isInit = false;
}

/* ===== ĐIỀU KHIỂN ĐỘNG CƠ (FIX LOGIC) ===== */
void motorsSetRatio(uint32_t id, uint16_t ratio)
{
    if (!isInit || id >= NBR_OF_MOTORS) return;

    uint32_t pulse_us;

    // Chuyển đổi Ratio (0 - 60000) sang Pulse (1000 - 2000 us)
    // MAX_THRUST = 60000 lấy từ motors.h
    if (ratio <= MIN_THRUST) {
        pulse_us = ESC_MIN_US;
    } else if (ratio >= MAX_THRUST) {
        pulse_us = ESC_MAX_US;
    } else {
        pulse_us = ESC_MIN_US + 
           ((uint32_t)ratio * (ESC_MAX_US - ESC_MIN_US)) / MAX_THRUST;
    }

    motor_us[id] = pulse_us;

    // Xác định Timer và Operator dựa trên ID
    // M1(0), M2(1) -> Timer 0
    // M3(2), M4(3) -> Timer 1
    mcpwm_timer_t timer_idx = (id < 2) ? MCPWM_TIMER_0 : MCPWM_TIMER_1;
    
    // M1(0), M3(2) -> Operator A
    // M2(1), M4(3) -> Operator B
    mcpwm_generator_t opr_idx = (id % 2 == 0) ? MCPWM_OPR_A : MCPWM_OPR_B;

    mcpwm_set_duty_in_us(MCPWM_UNIT_0, timer_idx, opr_idx, pulse_us);
}

int motorsGetRatio(uint32_t id)
{
    if (id >= NBR_OF_MOTORS) return -1;
    return motor_us[id];
}

/* ===== CÁC HÀM TIỆN ÍCH KHÁC ===== */

bool motorsTest(void)
{
    if (!isInit) return false;

    // Quay nhẹ để test thứ tự
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetRatio(i, 5000); // Khoảng 10-15% ga
    }
    vTaskDelay(pdMS_TO_TICKS(500)); 
    
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetRatio(i, 0);
    }
    return true;
}

// Các hàm âm thanh (ESC BLDC thường không hỗ trợ trực tiếp qua PWM motor)
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
    (void)id; (void)enable; (void)frequency; (void)ratio;
}

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec)
{
    (void)frequency;
    vTaskDelay(pdMS_TO_TICKS(duration_msec));
}

void motorsPlayMelody(uint16_t *notes)
{
    (void)notes;
}

/* ===== LOGGING ===== */
LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT32, m1, &motor_us[0])
LOG_ADD(LOG_UINT32, m2, &motor_us[1])
LOG_ADD(LOG_UINT32, m3, &motor_us[2])
LOG_ADD(LOG_UINT32, m4, &motor_us[3])
LOG_GROUP_STOP(pwm)
