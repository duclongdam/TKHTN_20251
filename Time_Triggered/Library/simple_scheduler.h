/* simple_scheduler.h */
#ifndef SIMPLE_SCHEDULER_H
#define SIMPLE_SCHEDULER_H

#include <stdint.h>

// Định nghĩa số lượng Task tối đa
#define MAX_TASKS 10

// Cấu trúc Task đơn giản hóa cho mô hình Simple Periodic
typedef struct {
    void (*pTask)(void); // Con trỏ hàm trỏ đến Task cần chạy
} sTask_t;

// Các hàm API
void SCH_Init(void);
uint8_t SCH_Add_Task(void (*task_func)(void)); // Không còn tham số thời gian
void SCH_Run_Cycle(void); // Vòng lặp chính
void SCH_Set_Wakeup_Callback(void (*callback_func)(void)); // Xử lý UART khi đang ngủ

#endif