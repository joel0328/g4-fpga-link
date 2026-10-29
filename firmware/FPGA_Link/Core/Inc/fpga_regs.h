/*
 * fpga_regs.h - register map for the Basys 3 link bitstream.
 * Keep in sync with rtl/regfile.v (map version 1).
 */
#ifndef FPGA_REGS_H
#define FPGA_REGS_H

#define FPGA_REG_ID          0x00u  /* RO   0x47344650 "G4FP"               */
#define FPGA_REG_VERSION     0x01u  /* RO   register map version            */
#define FPGA_REG_SCRATCH     0x02u  /* RW   free test register              */
#define FPGA_REG_LEDS        0x03u  /* RW   [15:0] LEDs                     */
#define FPGA_REG_SWITCHES    0x04u  /* RO   [15:0] switches, [20:16] buttons */
#define FPGA_REG_SEG7        0x05u  /* RW   [15:0] hex, [19:16] digit en, [20] use_reg */
#define FPGA_REG_TICK_DIV    0x06u  /* RW   LOOP_TICK period, 100 MHz clocks (0 = off) */
#define FPGA_REG_TICK_COUNT  0x07u  /* RO   ticks since reset               */
#define FPGA_REG_IRQ_STATUS  0x08u  /* RW1C IRQ status                      */
#define FPGA_REG_IRQ_ENABLE  0x09u  /* RW   IRQ enable                      */
#define FPGA_REG_FRAMES_OK   0x0Au  /* RO   good frames received            */
#define FPGA_REG_CRC_ERRS    0x0Bu  /* RO   frames rejected for bad CRC     */
#define FPGA_REG_TIME_US     0x0Cu  /* RO   free-running microseconds       */
#define FPGA_REG_CTRL        0x0Du  /* W    [0] clear link/tick counters    */

#define FPGA_ID_VALUE        0x47344650u
#define FPGA_MAP_VERSION     1u
#define FPGA_UNMAPPED_VALUE  0xDEADBEEFu

/* IRQ_STATUS / IRQ_ENABLE bits */
#define FPGA_IRQ_TICK        (1u << 0)
#define FPGA_IRQ_INPUTS      (1u << 1)
#define FPGA_IRQ_CRC_ERR     (1u << 2)

/* SEG7 fields */
#define FPGA_SEG7_USE_REG    (1u << 20)
#define FPGA_SEG7_ALL_DIGITS (0xFu << 16)

/* SWITCHES fields */
#define FPGA_SW_MASK         0x0000FFFFu
#define FPGA_BTN_U           (1u << 16)
#define FPGA_BTN_L           (1u << 17)
#define FPGA_BTN_R           (1u << 18)
#define FPGA_BTN_D           (1u << 19)
#define FPGA_BTN_C           (1u << 20)

#define FPGA_CTRL_CLEAR_COUNTERS (1u << 0)

/* LOOP_TICK helpers (FPGA system clock is 100 MHz) */
#define FPGA_CLK_HZ          100000000u
#define FPGA_TICK_DIV_FOR_HZ(hz) (FPGA_CLK_HZ / (hz))

#endif /* FPGA_REGS_H */
