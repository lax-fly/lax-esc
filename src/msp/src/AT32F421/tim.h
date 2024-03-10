#pragma once
#include <stdint.h>
#include "msp.h"

extern "C"
{
    void SysTick_Handler(void);
    void TMR14_GLOBAL_IRQHandler(void);
}

class Timer : public TimerIf
{
private:
    uint64_t exec_time;
    Task delay_task;
    Task timing_task;
    void *delay_data;
    void *timing_data;

    friend void SysTick_Handler(void);
    friend void TMR14_GLOBAL_IRQHandler(void);

public:
    Timer() : delay_task(nullptr), timing_task(nullptr) {}
    virtual void delay_us(uint16_t nus);
    virtual void delay_ms(uint16_t nms);
    virtual uint32_t now_ms(void) const;                             // return current time in ms
    virtual uint64_t now_us(void) const;                             // returncurrent time in us
    virtual void timing_task_10kHz(Task task, void *data);             // support only one task, so caller make sure that the last task is finished before next calling
};