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
 *   SPI1: PB3 SCK (D13), PB4 MISO (D12), PB5 MOSI (D11), with RX/TX DMA
 *
 * What it does:
 *   1. Init: reset the FPGA, check ID and map version.
 *   2. Self-test: 10,000 random write/readback pairs on SCRATCH, then
 *      2,000 16-word burst write/readback pairs on SCRATCH_RAM.
 *   3. Run: on every 1 kHz LOOP_TICK a control step runs entirely in
 *      interrupts over SPI DMA (see "Control step" below): LEDs follow the
 *      switches, the 7-segment display shows the tick count, pending FPGA
 *      IRQs are cleared. The main loop does no SPI; once a second it prints
 *      tick, link and control-step statistics.
 *      Press btnC on the Basys to re-run the self-tests.
 */
#include <stdio.h>
#include "fpga_link.h"
#include "fpga_link_app.h"

extern SPI_HandleTypeDef hspi1;

#define BURST_TEST_ITERATIONS 2000u   /* 2 x 2000 frames, 32,000 words each way */

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
/* Control step: one per LOOP_TICK, all in interrupts over SPI DMA          */
/*   tick ISR     -> burst read 0x04..0x0C (inputs, status, counters, time) */
/*   read done    -> burst write 0x03..0x05 (LEDS, SWITCHES = RO, SEG7)      */
/*   write done   -> if an enabled IRQ is pending, W1C it (1-word burst)    */
/* ------------------------------------------------------------------------ */
enum { IN_SWITCHES, IN_SEG7, IN_TICK_DIV, IN_TICK_COUNT, IN_IRQ_STATUS,
       IN_IRQ_ENABLE, IN_FRAMES_OK, IN_CRC_ERRS, IN_TIME_US, IN_N };
#define STEP_READ_ADDR   FPGA_REG_SWITCHES
#define STEP_WRITE_ADDR  FPGA_REG_LEDS
#define STEP_WRITE_N     3u   /* LEDS, SWITCHES (read-only: ignored), SEG7 */

typedef struct {
    uint32_t done, skipped, errors;
    uint32_t lat_min, lat_max;    /* tick edge -> last frame done, cycles */
    uint64_t lat_sum;
    uint64_t cpu_sum;             /* cycles spent in our tick/DMA callbacks */
    uint32_t in[IN_N];            /* last good read */
} step_stats_t;

static uint32_t step_in[IN_N];
static uint32_t step_out[STEP_WRITE_N];
static uint32_t step_irq_clear;
static volatile bool step_enabled;   /* off during init and self-tests */
static volatile bool step_active;    /* this step's frames are in flight */
static uint32_t step_t0, step_cpu;
static step_stats_t step_stats;      /* written in ISRs; read via step_snapshot() */

static void step_stats_reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    step_stats = (step_stats_t){ .lat_min = 0xFFFFFFFFu };
    __set_PRIMASK(primask);
}

static void step_snapshot(step_stats_t *s)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *s = step_stats;
    __set_PRIMASK(primask);
}

static void step_end(bool ok)
{
    if (ok) {
        uint32_t lat = DWT->CYCCNT - step_t0;
        step_stats.done++;
        if (lat < step_stats.lat_min) step_stats.lat_min = lat;
        if (lat > step_stats.lat_max) step_stats.lat_max = lat;
        step_stats.lat_sum += lat;
        step_stats.cpu_sum += step_cpu;
    } else {
        step_stats.errors++;
    }
    step_active = false;
}

static void step_written(fpga_link_t *l, fpga_status_t st)
{
    uint32_t c0 = DWT->CYCCNT;
    bool ok = (st == FPGA_OK), more = false;

    if (ok && step_irq_clear) {
        uint32_t v = step_irq_clear;   /* copied by the driver */
        step_irq_clear = 0;
        ok = more = (fpga_write_burst_async(l, FPGA_REG_IRQ_STATUS, &v, 1,
                                            step_written) == FPGA_OK);
    }
    step_cpu += DWT->CYCCNT - c0;
    if (!more)
        step_end(ok);
}

static void step_read_done(fpga_link_t *l, fpga_status_t st)
{
    uint32_t c0 = DWT->CYCCNT;
    bool ok = (st == FPGA_OK);

    if (ok) {
        /* The "control law": LEDs follow the switches, display shows ticks */
        step_out[0] = step_in[IN_SWITCHES] & FPGA_SW_MASK;
        step_out[1] = 0;
        step_out[2] = FPGA_SEG7_USE_REG | FPGA_SEG7_ALL_DIGITS |
                      (tick_isr_count & 0xFFFFu);
        step_irq_clear = step_in[IN_IRQ_STATUS] & step_in[IN_IRQ_ENABLE];
        for (uint32_t i = 0; i < IN_N; i++)
            step_stats.in[i] = step_in[i];
        ok = (fpga_write_burst_async(l, STEP_WRITE_ADDR, step_out, STEP_WRITE_N,
                                     step_written) == FPGA_OK);
    }
    step_cpu += DWT->CYCCNT - c0;
    if (!ok)
        step_end(false);
}

static void step_start(uint32_t now)
{
    if (!step_enabled)
        return;
    if (step_active) {            /* previous step still running */
        step_stats.skipped++;
        return;
    }
    step_active = true;
    step_t0 = now;
    step_cpu = 0;
    if (fpga_read_burst_async(&fpga, STEP_READ_ADDR, step_in, IN_N,
                              step_read_done) != FPGA_OK) {
        step_active = false;      /* link busy (main-loop transfer) or SPI error */
        step_stats.skipped++;
        return;
    }
    step_cpu += DWT->CYCCNT - now;
}

/* Stop starting steps and wait for the one in flight (main loop only) */
static void steps_pause(void)
{
    step_enabled = false;
    while (step_active)
        ;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == fpga.hspi)
        fpga_async_complete(&fpga, true);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == fpga.hspi)
        fpga_async_complete(&fpga, false);
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
        step_start(now);
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

    printf("Burst self-test: %lu x 16-word write/readback bursts...\r\n",
           (unsigned long)BURST_TEST_ITERATIONS);
    st = fpga_selftest_burst(&fpga, BURST_TEST_ITERATIONS, &r);
    printf("  %s: %lu iterations in %lu ms, mismatches %lu, MCU CRC errors %lu, "
           "FPGA CRC errors %lu, SPI errors %lu\r\n",
           fpga_status_str(st),
           (unsigned long)r.iterations, (unsigned long)r.elapsed_ms,
           (unsigned long)r.mismatches, (unsigned long)r.crc_errors,
           (unsigned long)r.fpga_crc_errors, (unsigned long)r.spi_errors);
    if (r.elapsed_ms)
        printf("  %lu register words/s (%lu burst frames/s)\r\n",
               (unsigned long)(2000u * FPGA_BURST_MAX * r.iterations / r.elapsed_ms),
               (unsigned long)(2000u * r.iterations / r.elapsed_ms));
}

void fpga_app_init(void)
{
    fpga_status_t st;

    step_enabled = false;
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

    fpga_write(&fpga, FPGA_REG_IRQ_STATUS, 0x7);
    fpga_write(&fpga, FPGA_REG_IRQ_ENABLE, FPGA_IRQ_INPUTS | FPGA_IRQ_CRC_ERR);

    /* Forget ticks that arrived from the previous run before fpga_init reset
     * the FPGA: they made the period max and the MCU/FPGA counts disagree.
     * The self-test cleared the FPGA's TICK_COUNT. */
    __disable_irq();
    tick_isr_count = 0;
    tick_min_cyc = 0xFFFFFFFFu;
    tick_max_cyc = 0;
    __enable_irq();
    step_stats_reset();

    /* 1 kHz loop tick, tick IRQ off (tick has its own line) */
    fpga_write(&fpga, FPGA_REG_TICK_DIV, FPGA_TICK_DIV_FOR_HZ(1000));
    step_enabled = true;
}

void fpga_app_poll(void)
{
    static uint32_t last_print;
    static uint32_t last_btn;
    step_stats_t s;

    if (!link_up) {
        /* Retry init every 2 s so the bitstream can be loaded after the MCU */
        if (HAL_GetTick() - last_print > 2000u) {
            last_print = HAL_GetTick();
            fpga_app_init();
        }
        return;
    }

    step_snapshot(&s);

    /* btnC: pause the control step and re-run the self-tests */
    uint32_t btn_c = s.in[IN_SWITCHES] & FPGA_BTN_C;
    if (btn_c && !last_btn) {
        steps_pause();
        run_selftest();
        step_enabled = true;
    }
    last_btn = btn_c;

    if (HAL_GetTick() - last_print >= 1000u) {
        last_print = HAL_GetTick();
        uint32_t cyc_per_us = SystemCoreClock / 1000000u;

        /* Tick period min/max in ns: shows LOOP_TICK + EXTI latency jitter */
        printf("ticks MCU %lu / FPGA %lu, period %lu..%lu ns, frames %lu, "
               "FPGA CRC errs %lu, MCU CRC errs %lu, IRQs %lu\r\n",
               (unsigned long)tick_isr_count, (unsigned long)s.in[IN_TICK_COUNT],
               (unsigned long)(tick_min_cyc * 1000u / cyc_per_us),
               (unsigned long)(tick_max_cyc * 1000u / cyc_per_us),
               (unsigned long)s.in[IN_FRAMES_OK], (unsigned long)s.in[IN_CRC_ERRS],
               (unsigned long)fpga.crc_errors, (unsigned long)irq_isr_count);

        if (s.done)
            printf("step: done %lu, skipped %lu, errors %lu, latency %lu..%lu ns "
                   "(avg %lu), callback CPU avg %lu ns\r\n",
                   (unsigned long)s.done, (unsigned long)s.skipped,
                   (unsigned long)s.errors,
                   (unsigned long)(s.lat_min * 1000u / cyc_per_us),
                   (unsigned long)(s.lat_max * 1000u / cyc_per_us),
                   (unsigned long)(s.lat_sum * 1000u / cyc_per_us / s.done),
                   (unsigned long)(s.cpu_sum * 1000u / cyc_per_us / s.done));
    }
}
