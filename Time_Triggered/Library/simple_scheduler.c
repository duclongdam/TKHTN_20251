/* simple_scheduler.c */
#include "simple_scheduler.h"
#include "main.h" 
#include <stdio.h>

// --- BIẾN NỘI BỘ ---
static sTask_t TaskList[MAX_TASKS]; 
static void (*WakeupCallback)(void) = NULL;
static uint8_t TaskCount = 0;
static volatile uint8_t flag_tick = 0;

extern TIM_HandleTypeDef htim2; // Sử dụng TIM2 làm nhịp tim (Heartbeat)

// Ngắt Timer xảy ra sau mỗi chu kỳ P (Do bạn cài đặt ARR)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        flag_tick = 1; // Báo hiệu đã đến lúc chạy chu kỳ mới
    }
}

void SCH_Init(void) {
    TaskCount = 0;
    flag_tick = 0;
    WakeupCallback = NULL;
    HAL_TIM_Base_Start_IT(&htim2); // Bắt đầu Timer
}

// Thêm Task vào danh sách
uint8_t SCH_Add_Task(void (*task_func)(void)) {
    if (TaskCount < MAX_TASKS) {
        TaskList[TaskCount].pTask = task_func;
        
        uint8_t current_id = TaskCount;
        TaskCount++;
        return current_id;
    }
    return 0xFF; // Lỗi: Danh sách đầy
}

// Đăng ký hàm xử lý khi bị đánh thức bởi nguồn khác (ví dụ UART)
void SCH_Set_Wakeup_Callback(void (*callback_func)(void)) {
    WakeupCallback = callback_func;
}

// Vòng lặp chính của bộ lập lịch
void SCH_Run_Cycle(void) {
    // 1. Kiểm tra xem đã đến chu kỳ P chưa?
    if (flag_tick) {
        flag_tick = 0; // Xóa cờ để chờ chu kỳ tiếp theo
        
        // --- SIMPLE PERIODIC LOGIC ---
        // Chạy tuần tự tất cả các task đã đăng ký
        for (int i = 0; i < TaskCount; i++) {
            if (TaskList[i].pTask != NULL) {
                TaskList[i].pTask(); 
            }
        }
    }

    // 2. Logic Ngủ (Sleep Mode) để tiết kiệm năng lượng
    // CPU sẽ ngủ cho đến khi có ngắt (Timer hoặc UART)
    while (flag_tick == 0)
    {
        HAL_SuspendTick(); 
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        HAL_ResumeTick(); 
        
        // Tỉnh dậy ở đây (do ngắt đánh thức)
        
        // Nếu tỉnh do UART (WakeupCallback), xử lý lệnh ngay
        if (WakeupCallback != NULL) {
            WakeupCallback();
        }
        
        // Nếu tỉnh do Timer, vòng while sẽ thoát (vì flag_tick = 1) 
        // và quay lại bước 1 để chạy các Task.
    }
}