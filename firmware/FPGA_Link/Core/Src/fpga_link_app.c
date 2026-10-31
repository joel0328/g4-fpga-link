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
 *      interrupts over SPI DMA (see "Control step" below): it reads the
 *      FPGA's tick snapshot, drives the LEDs from the switches and the
 *      7-segment display with the tick count, kicks the FPGA watchdog and
 *      clears pending FPGA IRQs. The main loop does no SPI while steps run;
 *      once a second it prints tick, link, control-step and safety status.
 *   4. Safety (map v4): the FPGA starts disarmed and the app never arms on
 *      its own. Arming is a deliberate action: btnC (re-runs the self-tests,
 *      clears faults, arms) or 'a' on the console. The FPGA also needs SW15
 *      up. A latched fault (missed kicks, link errors, btnD e-stop, SW15
 *      dropped) disarms in hardware; the app reports it and leaves it latched.
 *   5. Motors (map v5): the step writes the motor duties with each watchdog
 *      kick and reads both encoders with the snapshot. The duty comes only
 *      from the console ('m'), and the app drops it to 0 whenever the FPGA
 *      reports disarmed, so re-arming never restarts a motor. The FPGA clamps
 *      it to DUTY_LIMIT (50 % after reset).
 *
 * Console (VCP, 115200): a = arm, d = disarm, c = clear faults,
 *                        k = stop/restart watchdog kicks (to test the watchdog),
 *                        m<percent> then Enter or space = M1 duty, -100..100
 *                        (e.g. "m25 ", "m-40 "), s = stop both motors
 */
#include <stdio.h>
#include <stdlib.h>
#include "fpga_link.h"
#include "fpga_link_app.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

#define BURST_TEST_ITERATIONS 2000u   /* 2 x 2000 frames, 32,000 words each way */

/* Encoder counts per output-shaft revolution, for the RPM estimate only.
 * JGB37-520 12 V 178 RPM: usually 11 lines x4 x 1:56 gearbox = 2464, but
 * sellers vary; measure by turning the shaft one revolution by hand. */
#define ENC_COUNTS_PER_REV 2464

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
/*   tick ISR       -> burst read SNAP_* 0x30..0x37 (one coherent sample,   */
/*                     encoders included)                                   */
/*   snapshot done  -> burst write 0x03..0x05 (LEDS, SWITCHES = RO, SEG7)   */
/*   outputs done   -> burst write 0x24..0x26: WDT_KICK, M1_DUTY, M2_DUTY   */
/*                     (0x25..0x26 only while kicks are stopped with 'k')   */
/*   kick done      -> burst read 0x25..0x28 (duties, limit, encoder errs)  */
/*   motor done     -> burst read 0x07..0x0B (tick count, IRQs, link stats) */
/*   status done    -> if an enabled IRQ is pending, W1C it                 */
/* ------------------------------------------------------------------------ */
enum { SNAP_TICK, SNAP_TIME_US, SNAP_INPUTS, SNAP_SAFETY,
       SNAP_ENC1, SNAP_ENC1_DELTA, SNAP_ENC2, SNAP_ENC2_DELTA, SNAP_N };
enum { ST_TICK_COUNT, ST_IRQ_STATUS, ST_IRQ_ENABLE, ST_FRAMES_OK, ST_CRC_ERRS, ST_N };
enum { MOT_M1_DUTY, MOT_M2_DUTY, MOT_LIMIT, MOT_ENC_ERRS, MOT_N };
#define STEP_STATUS_ADDR FPGA_REG_TICK_COUNT
#define STEP_WRITE_ADDR  FPGA_REG_LEDS
#define STEP_WRITE_N     3u   /* LEDS, SWITCHES (read-only: ignored), SEG7 */

enum { STAGE_SNAP, STAGE_OUT, STAGE_KICK, STAGE_MOTOR, STAGE_STATUS, STAGE_IRQ_CLEAR };

typedef struct {
    uint32_t done, skipped, errors;
    uint32_t lat_min, lat_max;    /* tick edge -> last frame done, cycles */
    uint64_t lat_sum;
    uint64_t cpu_sum;             /* cycles spent in our tick/DMA callbacks */
    uint32_t snap[SNAP_N];        /* last good snapshot */
    uint32_t status[ST_N];        /* last good status read */
    uint32_t motor[MOT_N];        /* last good motor readback */
} step_stats_t;

static uint32_t step_snap[SNAP_N];
static uint32_t step_status[ST_N];
static uint32_t step_out[STEP_WRITE_N];
static uint32_t step_kick_out[3];    /* WDT_KICK, M1_DUTY, M2_DUTY */
static uint32_t step_motor[MOT_N];
static volatile int16_t motor_cmd[2];   /* duty commands, +/-FPGA_DUTY_MAX */
static uint32_t step_stage;
static volatile bool step_enabled;   /* off during init and self-tests */
static volatile bool step_active;    /* this step's frames are in flight */
static volatile bool step_kick = true;
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

/* DMA completion of each frame in the step: start the next one, or finish */
static void step_next(fpga_link_t *l, fpga_status_t st)
{
    uint32_t c0 = DWT->CYCCNT;
    bool ok = (st == FPGA_OK), more = false;
    fpga_status_t r = FPGA_OK;
    uint32_t v = 0;               /* single words are copied by the driver */

    if (ok) {
        switch (step_stage) {
        case STAGE_SNAP:
            /* The "control law": LEDs follow the switches, display shows ticks */
            for (uint32_t i = 0; i < SNAP_N; i++)
                step_stats.snap[i] = step_snap[i];
            step_out[0] = step_snap[SNAP_INPUTS] & FPGA_LEDS_VALUE;
            step_out[1] = 0;
            step_out[2] = FPGA_SEG7_USE_REG | FPGA_SEG7_ALL_DIGITS |
                          (step_snap[SNAP_TICK] & 0xFFFFu);
            /* Disarmed: forget the duty, so arming never restarts a motor */
            if (!(step_snap[SNAP_SAFETY] & FPGA_ARM_ARMED))
                motor_cmd[0] = motor_cmd[1] = 0;
            step_stage = STAGE_OUT;
            more = true;
            r = fpga_write_burst_async(l, STEP_WRITE_ADDR, step_out, STEP_WRITE_N, step_next);
            break;
        case STAGE_OUT:
            step_kick_out[0] = 0;
            step_kick_out[1] = (uint16_t)motor_cmd[0];
            step_kick_out[2] = (uint16_t)motor_cmd[1];
            step_stage = STAGE_KICK;
            more = true;
            if (step_kick)
                r = fpga_write_burst_async(l, FPGA_REG_WDT_KICK, step_kick_out, 3, step_next);
            else
                r = fpga_write_burst_async(l, FPGA_REG_M1_DUTY, &step_kick_out[1], 2, step_next);
            break;
        case STAGE_KICK:
            step_stage = STAGE_MOTOR;
            more = true;
            r = fpga_read_burst_async(l, FPGA_REG_M1_DUTY, step_motor, MOT_N, step_next);
            break;
        case STAGE_MOTOR:
            for (uint32_t i = 0; i < MOT_N; i++)
                step_stats.motor[i] = step_motor[i];
            step_stage = STAGE_STATUS;
            more = true;
            r = fpga_read_burst_async(l, STEP_STATUS_ADDR, step_status, ST_N, step_next);
            break;
        case STAGE_STATUS:
            for (uint32_t i = 0; i < ST_N; i++)
                step_stats.status[i] = step_status[i];
            v = step_status[ST_IRQ_STATUS] & step_status[ST_IRQ_ENABLE];
            if (v) {
                step_stage = STAGE_IRQ_CLEAR;
                more = true;
                r = fpga_write_burst_async(l, FPGA_REG_IRQ_STATUS, &v, 1, step_next);
            }
            break;
        default:
            break;
        }
        ok = (r == FPGA_OK);
    }
    step_cpu += DWT->CYCCNT - c0;
    if (!(more && ok))
        step_end(ok);
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
    step_stage = STAGE_SNAP;
    if (fpga_read_burst_async(&fpga, FPGA_REG_SNAP_TICK, step_snap, SNAP_N,
                              step_next) != FPGA_OK) {
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

/* Restart the MCU's tick statistics. Call right after clearing the FPGA's
 * TICK_COUNT so the two counts line up. */
static void tick_stats_reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    tick_isr_count = 0;
    tick_min_cyc = 0xFFFFFFFFu;
    tick_max_cyc = 0;
    __set_PRIMASK(primask);
}

/* ------------------------------------------------------------------------ */
/* Safety (map v4). These use blocking frames, so call them from the main   */
/* loop with the control step paused (see safety_cmd).                      */
/* ------------------------------------------------------------------------ */
static void print_faults(uint32_t f)
{
    printf("%s%s%s%s%s",
           f ? "" : " none",
           (f & FPGA_FAULT_WDT)      ? " WDT(missed kicks)" : "",
           (f & FPGA_FAULT_LINK)     ? " LINK(rejected frames)" : "",
           (f & FPGA_FAULT_ESTOP)    ? " ESTOP(btnD)" : "",
           (f & FPGA_FAULT_ARM_LOST) ? " ARM_LOST(SW15 down)" : "");
}

/* Ask the FPGA to arm, then say whether it did (and if not, why) */
static void safety_arm(void)
{
    uint32_t arm = 0, faults = 0;

    step_kick = true;
    fpga_write(&fpga, FPGA_REG_ARM, FPGA_ARM_KEY);
    fpga_read(&fpga, FPGA_REG_ARM, &arm);
    if (arm & FPGA_ARM_ARMED) {
        printf("safety: armed\r\n");
        return;
    }
    fpga_read(&fpga, FPGA_REG_FAULT_STATUS, &faults);
    printf("safety: arm refused:%s%s%s\r\n",
           (arm & FPGA_ARM_SW_EN) ? "" : " SW15 is down;",
           (arm & FPGA_ARM_TICK_ON) ? "" : " tick not running;",
           (arm & FPGA_ARM_FAULT) ? " fault latched (c clears):" : "");
    if (arm & FPGA_ARM_FAULT) {
        printf("  faults:");
        print_faults(faults);
        printf("\r\n");
    }
}

static void safety_disarm(void)
{
    fpga_write(&fpga, FPGA_REG_ARM, 0);
}

static void safety_clear_faults(void)
{
    uint32_t faults = 0;

    fpga_write(&fpga, FPGA_REG_FAULT_CLEAR, FPGA_FAULT_WDT | FPGA_FAULT_LINK |
                                            FPGA_FAULT_ESTOP | FPGA_FAULT_ARM_LOST);
    fpga_read(&fpga, FPGA_REG_FAULT_STATUS, &faults);
    printf("safety: faults cleared; still latched:");
    print_faults(faults);
    printf("\r\n");
}

/* Run one console command with the control step paused, so its blocking
 * frames don't collide with the step's DMA frames */
static void safety_cmd(char c)
{
    steps_pause();
    switch (c) {
    case 'a': safety_arm(); break;
    case 'd': safety_disarm(); printf("safety: disarmed\r\n"); break;
    case 'c': safety_clear_faults(); break;
    case 'k':
        step_kick = !step_kick;
        printf("safety: watchdog kicks %s\r\n", step_kick ? "on" : "OFF (expect FAULT WDT)");
        break;
    default: break;
    }
    step_enabled = true;
}

/* "m<percent>": set M1's duty (main loop; the next step sends it) */
static void motor_set(const char *arg, bool armed)
{
    long pct = strtol(arg, NULL, 10);

    if (!armed) {
        printf("motor: not armed; arm first (SW15 up, then 'a' or btnC)\r\n");
        return;
    }
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;
    motor_cmd[0] = (int16_t)(pct * (long)FPGA_DUTY_MAX / 100);
    printf("motor: M1 %ld %% (duty %d); the FPGA clamps to DUTY_LIMIT\r\n",
           pct, (int)motor_cmd[0]);
}

static void motor_stop(void)
{
    motor_cmd[0] = motor_cmd[1] = 0;
    printf("motor: stopped\r\n");
}

/* One console byte, if any (polled; the VCP is otherwise only written) */
static int console_getc(void)
{
    USART_TypeDef *u = huart2.Instance;

    if (u->ISR & USART_ISR_ORE)
        u->ICR = USART_ICR_ORECF;
    if (u->ISR & USART_ISR_RXNE_RXFNE)
        return (int)(u->RDR & 0xFFu);
    return -1;
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

    fpga_write(&fpga, FPGA_REG_IRQ_STATUS, 0xF);
    fpga_write(&fpga, FPGA_REG_IRQ_ENABLE,
               FPGA_IRQ_INPUTS | FPGA_IRQ_CRC_ERR | FPGA_IRQ_FAULT);

    /* Forget ticks that arrived from the previous run before fpga_init reset
     * the FPGA: they made the period max and the MCU/FPGA counts disagree.
     * The self-test cleared the FPGA's TICK_COUNT. */
    tick_stats_reset();
    step_stats_reset();

    /* Start the 1 kHz loop tick (tick IRQ off: it has its own line) and the
     * control step. The FPGA stays disarmed until the user arms it. */
    fpga_write(&fpga, FPGA_REG_TICK_DIV, FPGA_TICK_DIV_FOR_HZ(1000));
    step_enabled = true;
    printf("safety: disarmed; SW15 up, then btnC or 'a' to arm\r\n");
    printf("console: a arm, d disarm, c clear faults, k kicks, m<pct> M1 duty, s stop\r\n");
}

void fpga_app_poll(void)
{
    static uint32_t last_print;
    static uint32_t last_btn;
    static uint32_t last_faults;
    static uint32_t last_enc1, last_enc_tick;
    static char mline[8];
    static int mlen = -1;            /* >= 0 while reading "m<percent>" */
    step_stats_t s;
    int c;

    if (!link_up) {
        /* Retry init every 2 s so the bitstream can be loaded after the MCU */
        if (HAL_GetTick() - last_print > 2000u) {
            last_print = HAL_GetTick();
            fpga_app_init();
        }
        return;
    }

    step_snapshot(&s);
    uint32_t safety = s.snap[SNAP_SAFETY];

    if ((c = console_getc()) >= 0) {
        if (mlen >= 0) {
            if (((c >= '0' && c <= '9') || (c == '-' && mlen == 0)) &&
                mlen < (int)sizeof mline - 1) {
                mline[mlen++] = (char)c;
            } else {
                mline[mlen] = '\0';
                if ((c == '\r' || c == '\n' || c == ' ') && mlen > 0)
                    motor_set(mline, (safety & FPGA_ARM_ARMED) != 0);
                else
                    printf("motor: usage m<percent> then Enter, e.g. m25 or m-40\r\n");
                mlen = -1;
            }
        } else if (c == 'm') {
            mlen = 0;
        } else if (c == 's') {
            motor_stop();
        } else if (c != '\r' && c != '\n' && c != ' ') {
            safety_cmd((char)c);
        }
    }

    uint32_t faults = (safety >> FPGA_SNAP_FAULT_LSB) & 0xFu;

    /* btnC: pause the control step, re-run the self-tests, clear faults, arm */
    uint32_t btn_c = s.snap[SNAP_INPUTS] & FPGA_BTN_C;
    if (btn_c && !last_btn) {
        steps_pause();
        safety_disarm();
        run_selftest();
        fpga_write(&fpga, FPGA_REG_CTRL, FPGA_CTRL_CLEAR_COUNTERS);
        tick_stats_reset();
        safety_clear_faults();
        safety_arm();
        step_enabled = true;
    }
    last_btn = btn_c;

    /* Report each newly latched fault once; it stays latched until the user
     * clears it (btnC or 'c') */
    if (faults & ~last_faults) {
        printf("SAFETY FAULT:");
        print_faults(faults & ~last_faults);
        printf("; outputs off. Clear with btnC or 'c', then arm\r\n");
    }
    last_faults = faults;

    if (HAL_GetTick() - last_print >= 1000u) {
        last_print = HAL_GetTick();
        uint32_t cyc_per_us = SystemCoreClock / 1000000u;

        /* Tick period min/max in ns: shows LOOP_TICK + EXTI latency jitter */
        printf("ticks MCU %lu / FPGA %lu, period %lu..%lu ns, frames %lu, "
               "FPGA CRC errs %lu, MCU CRC errs %lu, IRQs %lu\r\n",
               (unsigned long)tick_isr_count, (unsigned long)s.status[ST_TICK_COUNT],
               (unsigned long)(tick_min_cyc * 1000u / cyc_per_us),
               (unsigned long)(tick_max_cyc * 1000u / cyc_per_us),
               (unsigned long)s.status[ST_FRAMES_OK], (unsigned long)s.status[ST_CRC_ERRS],
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

        printf("safety: %s, SW15 %s, kicks %s, faults 0x%lX\r\n",
               (safety & FPGA_ARM_ARMED) ? "armed" : "disarmed",
               (safety & FPGA_ARM_SW_EN) ? "up" : "down",
               step_kick ? "on" : "off",
               (unsigned long)faults);

        /* Speed from the encoder count change over the ticks since the last
         * line (1 tick = 1 ms) */
        int32_t enc1 = (int32_t)s.snap[SNAP_ENC1];
        uint32_t dt = s.snap[SNAP_TICK] - last_enc_tick;
        int32_t cps = dt ? (int32_t)((int64_t)(int32_t)(s.snap[SNAP_ENC1] - last_enc1) * 1000 / (int32_t)dt) : 0;
        last_enc1 = s.snap[SNAP_ENC1];
        last_enc_tick = s.snap[SNAP_TICK];
        printf("motor: M1 duty %ld (FPGA %ld), limit %lu, enc1 %ld, %ld counts/s "
               "(~%ld RPM at %d/rev), enc2 %ld, enc errs %lu/%lu\r\n",
               (long)motor_cmd[0], (long)(int32_t)s.motor[MOT_M1_DUTY],
               (unsigned long)s.motor[MOT_LIMIT], (long)enc1, (long)cps,
               (long)(cps * 60 / ENC_COUNTS_PER_REV), ENC_COUNTS_PER_REV,
               (long)(int32_t)s.snap[SNAP_ENC2],
               (unsigned long)(s.motor[MOT_ENC_ERRS] & FPGA_ENC_ERRS_ENC1),
               (unsigned long)(s.motor[MOT_ENC_ERRS] >> 16));
    }
}
