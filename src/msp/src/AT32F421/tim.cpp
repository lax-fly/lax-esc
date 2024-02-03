#include "tim.h"
#include "at32f421_tmr.h"

volatile uint32_t sys_ticks;
uint32_t tim6_overflow = 0;

static Timer timer;
static bool timer_inited = false;

#define SYSTICK_CLOCK_DIV 8
#define SYSTICK_CLOCK (system_core_clock / SYSTICK_CLOCK_DIV)

void systick_init(void)
{
    SysTick_Config(SYSTICK_CLOCK / 1000);
    systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_DIV8);
    nvic_irq_enable(SysTick_IRQn, 0, 0);
}

void tmr6_init(void)
{
    crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK, TRUE);
    tmr_base_init(TMR6, 65535, 119);
    tmr_cnt_dir_set(TMR6, TMR_COUNT_UP);
    tmr_period_buffer_enable(TMR6, FALSE);
    tmr_primary_mode_select(TMR6, TMR_PRIMARY_SEL_OVERFLOW);
    tmr_overflow_request_source_set(TMR6, TRUE);
    tmr_counter_enable(TMR6, TRUE);
    tmr_interrupt_enable(TMR6, TMR_OVF_INT, TRUE);
    nvic_irq_enable(TMR6_GLOBAL_IRQn, 0, 0);
}

void tmr14_init(void)
{
    crm_periph_clock_enable(CRM_TMR14_PERIPH_CLOCK, TRUE);
    tmr_base_init(TMR14, 49, 119);
    tmr_cnt_dir_set(TMR14, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR14, TMR_CLOCK_DIV1);
    tmr_period_buffer_enable(TMR14, FALSE);
    tmr_overflow_request_source_set(TMR14, TRUE);
    tmr_interrupt_enable(TMR14, TMR_OVF_INT, TRUE);
    nvic_irq_enable(TMR14_GLOBAL_IRQn, 0, 0);
}

TimerIf *TimerIf::singleton()
{
    if (timer_inited)
        return &timer;
    timer_inited = true;
    systick_init();
    tmr6_init();
    tmr14_init();
    return &timer;
}

void Timer::delay_us(uint16_t nus)
{
    uint16_t te = TMR6->cval + nus;
    while (TMR6->cval > te) // te overflowed
    {
    }
    while (TMR6->cval < te)
    {
    }
}

void Timer::delay_ms(uint16_t nms)
{
    uint32_t stop = sys_ticks + nms;
    while (sys_ticks < stop)
    {
    }
}

uint32_t Timer::now_ms(void) const
{
    return sys_ticks;
}

uint64_t Timer::now_us(void) const
{
    return ((uint64_t)tim6_overflow << 16) + (uint64_t)TMR6->cval;
}

void Timer::timing_task_1ms(Task task, void *data)
{
    timing_data = data; // must set data before task, or, there is risk that data is not set when task is called by SysTick_Handler
    timing_task = task;
}

void Timer::delay_task_us(uint32_t nus, Task task, void *data)
{
    delay_data = data;
    delay_task = task;
    TMR14->cval = 0;            // clear counter
    TMR14->ctrl1_bit.tmren = 1; // enable counter
}

extern "C"
{
    void SysTick_Handler(void) // 1ms
    {
        sys_ticks++;
        if (timer.timing_task)
            timer.timing_task(timer.timing_data);
    }
    void TMR6_GLOBAL_IRQHandler(void) // work as time base
    {
        TMR6->ists = 0;
        tim6_overflow++;
    }
    void TMR14_GLOBAL_IRQHandler(void) // to exec delay task
    {
        TMR14->ists = 0;
        TMR14->ctrl1_bit.tmren = 0; // disable counter
        if (timer.delay_task)
            timer.delay_task(timer.delay_data);
    }
}
