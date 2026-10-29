/*
 * fpga_link_app.c - bring-up application for the Nucleo-G431KB <-> Basys 3 link.
 *
 * Call fpga_app_init() once after MX_*_Init() in main(), then fpga_app_poll()
 * from the while(1) loop. Output goes to printf (retarget to USART2 / VCP,
 * see README). Requires these CubeMX user labels:
 *
 *   FPGA_CS      PA11 (D10)  GPIO_Output, initial High
 *   FPGA_RST_N   PB6  (D6)   GPIO_Output, initial High
 *   FPGA_IRQ     PA12 (D2)   GPIO_EXTI12, rising edge, pull-down
 *   FPGA_TICK    PB0  (D3)   GPIO_EXTI0,  rising edge, pull-down
 *   FPGA_FAULT_N PA8  (D9)   GPIO_Input,  pull-up
 *   SPI1: PB3 SCK (D13), PB4 MISO (D12), PB5 MOSI (D11)
 *
 * What it does:
 *   1. Init: reset the FPGA, check ID and map version.
 *   2. Self-test: 10,000 random write/readback pairs on SCRATCH.
 *   3. Run loop: LEDs mirror the switches; FPGA LOOP_TICK runs at 1 kHz and
 *      its count is shown on the 7-segment display; once a second the app
 *      prints tick count, tick jitter and link statistics.
 *      Press btnC on the Basys to re-run the self-test.
 */
#include <stdio.h>
#include "fpga_link.h"
#include "fpga_link_app.h"

extern SPI_HandleTypeDef hspi1;

static fpga_link_t fpga = {
    .hspi       = &hspi1,
    .cs_port    = FPGA_CS_GPIO_Port,
    .cs_pin     = FPGA_CS_Pin,
    .rst_port   = FPGA_RST_N_GPIO_Port,
    .rst_pin    = FPGA_RST_N_Pin,
    .timeout_ms = 5,
    .retries    = 2,
};

static volatile uint32_t tick_isr_count;
static volatile uint32_t irq_isr_count;
static volatile uint32_t tick_last_cyc, tick_min_cyc = 0xFFFFFFFFu, tick_max_cyc;
static bool link_up;

bool fpga_app_link_up(void)
{
    return link_up;
}

/* ------------------------------------------------------------------------ */
/* EXTI: LOOP_TICK and FPGA_IRQ                                             */
/* ------------------------------------------------------------------------ */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == FPGA_TICK_Pin) {
        /* Period measured with the DWT cycle counter (170 MHz) */
        uint32_t now = DWT->CYCCNT;
        if (tick_isr_count) {
            uint32_t d = now - tick_last_cyc;
            if (d < tick_min_cyc) tick_min_cyc = d;
            if (d > tick_max_cyc) tick_max_cyc = d;
        }
        tick_last_cyc = now;
        tick_isr_count++;
        /* A control loop would run here: read sensors, step(), write outputs */
    } else if (pin == FPGA_IRQ_Pin) {
        irq_isr_count++;
    }
}

static void dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ------------------------------------------------------------------------ */
static void run_selftest(void)
{
    fpga_selftest_t r;
    fpga_status_t st;

    printf("Self-test: 10000 write/readback pairs...\r\n");
    st = fpga_selftest(&fpga, 10000, &r);
    printf("  %s: %lu iterations in %lu ms, mismatches %lu, MCU CRC errors %lu, "
           "FPGA CRC errors %lu, SPI errors %lu\r\n",
           fpga_status_str(st),
           (unsigned long)r.iterations, (unsigned long)r.elapsed_ms,
           (unsigned long)r.mismatches, (unsigned long)r.crc_errors,
           (unsigned long)r.fpga_crc_errors, (unsigned long)r.spi_errors);
    if (r.elapsed_ms)
        printf("  %lu register transactions/s\r\n",
               (unsigned long)(2000u * r.iterations / r.elapsed_ms));
}

void fpga_app_init(void)
{
    fpga_status_t st;

    dwt_enable();
    printf("\r\n=== FPGA link bring-up ===\r\n");
    printf("SPI1 clock: %lu Hz\r\n",
           (unsigned long)(HAL_RCC_GetPCLK2Freq() >>
                           (((hspi1.Init.BaudRatePrescaler >> SPI_CR1_BR_Pos) & 7u) + 1u)));

    st = fpga_init(&fpga);
    printf("Init: %s\r\n", fpga_status_str(st));
    if (st != FPGA_OK) {
        printf("  Check: bitstream loaded, JA wiring, common ground, SPI mode 0.\r\n");
        printf("  FAULT_N pin reads %d\r\n",
               (int)HAL_GPIO_ReadPin(FPGA_FAULT_N_GPIO_Port, FPGA_FAULT_N_Pin));
        link_up = false;
        return;
    }
    link_up = true;

    run_selftest();

    /* 1 kHz loop tick, tick IRQ off (tick has its own line) */
    fpga_write(&fpga, FPGA_REG_IRQ_STATUS, 0x7);
    fpga_write(&fpga, FPGA_REG_IRQ_ENABLE, FPGA_IRQ_INPUTS | FPGA_IRQ_CRC_ERR);
    fpga_write(&fpga, FPGA_REG_TICK_DIV, FPGA_TICK_DIV_FOR_HZ(1000));
}

void fpga_app_poll(void)
{
    static uint32_t last_print;
    static uint32_t last_btn;
    static uint32_t last_irq;
    uint32_t sw, v;

    if (!link_up) {
        /* Retry init every 2 s so the bitstream can be loaded after the MCU */
        if (HAL_GetTick() - last_print > 2000u) {
            last_print = HAL_GetTick();
            fpga_app_init();
        }
        return;
    }

    /* LEDs mirror switches */
    if (fpga_read(&fpga, FPGA_REG_SWITCHES, &sw) == FPGA_OK) {
        fpga_write(&fpga, FPGA_REG_LEDS, sw & FPGA_SW_MASK);

        uint32_t btn_c = sw & FPGA_BTN_C;
        if (btn_c && !last_btn)
            run_selftest();
        last_btn = btn_c;
    }

    /* Tick count on the display */
    fpga_write(&fpga, FPGA_REG_SEG7,
               FPGA_SEG7_USE_REG | FPGA_SEG7_ALL_DIGITS | (tick_isr_count & 0xFFFFu));

    /* FPGA_IRQ is a level (status & enable) and the EXTI fires on its rising
     * edge, so clear the status to re-arm it. Checking the pin level (not just
     * the ISR count) also catches an event that lands while we are clearing. */
    if (irq_isr_count != last_irq ||
        HAL_GPIO_ReadPin(FPGA_IRQ_GPIO_Port, FPGA_IRQ_Pin) == GPIO_PIN_SET) {
        last_irq = irq_isr_count;
        if (fpga_read(&fpga, FPGA_REG_IRQ_STATUS, &v) == FPGA_OK)
            fpga_write(&fpga, FPGA_REG_IRQ_STATUS, v);
    }

    if (HAL_GetTick() - last_print >= 1000u) {
        uint32_t fpga_ticks = 0, frames = 0, crc_errs = 0;
        last_print = HAL_GetTick();
        fpga_read(&fpga, FPGA_REG_TICK_COUNT, &fpga_ticks);
        fpga_read(&fpga, FPGA_REG_FRAMES_OK, &frames);
        fpga_read(&fpga, FPGA_REG_CRC_ERRS, &crc_errs);

        /* Tick period min/max in ns: shows LOOP_TICK + EXTI latency jitter */
        uint32_t cyc_per_us = SystemCoreClock / 1000000u;
        printf("ticks MCU %lu / FPGA %lu, period %lu..%lu ns, frames %lu, "
               "FPGA CRC errs %lu, MCU CRC errs %lu, IRQs %lu\r\n",
               (unsigned long)tick_isr_count, (unsigned long)fpga_ticks,
               (unsigned long)(tick_min_cyc * 1000u / cyc_per_us),
               (unsigned long)(tick_max_cyc * 1000u / cyc_per_us),
               (unsigned long)frames, (unsigned long)crc_errs,
               (unsigned long)fpga.crc_errors, (unsigned long)irq_isr_count);
    }
}
