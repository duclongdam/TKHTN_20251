#include "queue_handle.h"

#define QUEUE_SIZE 8
#define NO_TASK 0

uint8_t task_queue[QUEUE_SIZE];
volatile int8_t head = 0;
volatile int8_t tail = 0;

bool Push_Queue(uint8_t task_id) {
   uint8_t next_tail = (tail + 1) % QUEUE_SIZE;
   if (next_tail == head) {
       return false; //Queue full
   }
   task_queue[tail] = task_id;
   tail = next_tail;
   return true;
}

uint8_t Pop_Queue(void) {
   if (head == tail) {
       return NO_TASK;
   }
   uint8_t task_id = task_queue[head];
   head = (head + 1) % QUEUE_SIZE;
   return task_id;
}

