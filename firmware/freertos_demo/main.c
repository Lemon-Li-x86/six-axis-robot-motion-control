#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"


/* =========================================================
 * UART0 寄存器
 * 与之前裸机 hello 工程使用的是同一个 UART。
 * ========================================================= */

#define UART0_BASE 0x40004000UL

#define UART0_DATA \
    (*(volatile uint32_t *)(UART0_BASE + 0x000UL))

#define UART0_STATE \
    (*(volatile uint32_t *)(UART0_BASE + 0x004UL))

#define UART0_CTRL \
    (*(volatile uint32_t *)(UART0_BASE + 0x008UL))

#define UART0_BAUDDIV \
    (*(volatile uint32_t *)(UART0_BASE + 0x010UL))

#define UART_TX_FULL (1U << 0)


/* =========================================================
 * UART 基础函数
 * ========================================================= */

/* 初始化 UART0 */
static void uart_init(void)
{
    /* 设置波特率分频值 */
    UART0_BAUDDIV = 16U;

    /* CTRL bit0 = 1：开启发送功能 */
    UART0_CTRL = 1U;
}


/* 发送单个字符 */
static void uart_putc(char c)
{
    /* 等待 UART 发送缓冲区有空位 */
    while (UART0_STATE & UART_TX_FULL)
    {
    }

    UART0_DATA = (uint32_t)c;
}


/* 发送字符串 */
static void uart_puts(const char *s)
{
    while (*s)
    {
        uart_putc(*s++);
    }
}


/* =========================================================
 * FreeRTOS Task A
 * ========================================================= */

static void task_a(void *parameters)
{
    /* 当前 demo 不需要任务参数 */
    (void)parameters;

    for (;;)
    {
        uart_puts("Task A running\r\n");

        /*
         * 当前 Tick Rate = 1000 Hz，
         * pdMS_TO_TICKS(1000) 表示延时约 1000 ms。
         *
         * vTaskDelay() 会让当前任务进入 Blocked 状态，
         * FreeRTOS 可以在此期间运行其他任务。
         */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


/* =========================================================
 * FreeRTOS Task B
 * ========================================================= */

static void task_b(void *parameters)
{
    (void)parameters;

    for (;;)
    {
        uart_puts("Task B running\r\n");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


/* =========================================================
 * 程序入口
 * ========================================================= */

int main(void)
{
    /* 初始化 UART，供两个任务输出调试信息 */
    uart_init();

    uart_puts("Starting FreeRTOS...\r\n");


    /*
     * 创建 Task A。
     *
     * 参数依次表示：
     *
     * task_a
     *     任务执行函数
     *
     * "TaskA"
     *     调试时显示的任务名称
     *
     * configMINIMAL_STACK_SIZE
     *     给任务分配的栈大小
     *
     * NULL
     *     不向任务传递参数
     *
     * 1
     *     任务优先级
     *
     * NULL
     *     当前不保存 Task Handle
     */
    xTaskCreate(
        task_a,
        "TaskA",
        configMINIMAL_STACK_SIZE,
        NULL,
        1,
        NULL
    );


    /* 创建 Task B */
    xTaskCreate(
        task_b,
        "TaskB",
        configMINIMAL_STACK_SIZE,
        NULL,
        1,
        NULL
    );


    /*
     * 启动 FreeRTOS 调度器。
     *
     * 正常情况下，一旦启动成功，
     * 程序就不会再回到 main()。
     */
    vTaskStartScheduler();


    /*
     * 如果代码执行到这里，
     * 一般说明调度器没有成功启动。
     *
     * 最常见原因之一是创建 Idle Task 时内存不足。
     */
    uart_puts("ERROR: Scheduler failed!\r\n");

    while (1)
    {
    }
}