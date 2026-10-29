/*
 * fpga_link.h - STM32 HAL driver for the CRC-8 framed SPI register link.
 *
 * SPI settings (CubeMX, SPI1 on the Nucleo-G431KB):
 *   Mode: Full-Duplex Master, Hardware NSS: Disable (CS is a GPIO)
 *   Data size 8 bits, MSB first, CPOL Low, CPHA 1 Edge (mode 0)
 *   Prescaler 16 (10.6 MHz); 32 (5.3 MHz) also supported
 *   CRC calculation: Disabled (the protocol carries its own CRC)
 *
 * Frame format and CRC are described in rtl/spi_link.v.
 */
#ifndef FPGA_LINK_H
#define FPGA_LINK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef FPGA_LINK_HOST_TEST
#include "hal_stub.h"
#else
#include "main.h"   /* CubeMX: pulls in the stm32g4xx HAL */
#endif

#include "fpga_regs.h"

typedef enum {
    FPGA_OK = 0,
    FPGA_ERR_SPI,       /* HAL SPI call failed or timed out            */
    FPGA_ERR_CRC,       /* response CRC mismatch (after retries)        */
    FPGA_ERR_VERIFY,    /* write readback did not match                 */
    FPGA_ERR_ID,        /* ID register did not read FPGA_ID_VALUE       */
    FPGA_ERR_VERSION,   /* register map version mismatch                */
    FPGA_ERR_PARAM,     /* bad argument (burst count 0 or > 16)         */
} fpga_status_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    GPIO_TypeDef      *rst_port;   /* FPGA_RST_N, may be NULL            */
    uint16_t           rst_pin;
    uint32_t           timeout_ms; /* per HAL transfer; 0 -> default 5   */
    uint8_t            retries;    /* extra attempts on CRC/SPI error    */

    /* statistics, maintained by the driver */
    uint32_t reads;
    uint32_t writes;
    uint32_t crc_errors;
    uint32_t spi_errors;
} fpga_link_t;

typedef struct {
    uint32_t iterations;
    uint32_t mismatches;       /* scratch readback != written value       */
    uint32_t crc_errors;       /* MCU-side response CRC failures          */
    uint32_t fpga_crc_errors;  /* frames the FPGA rejected (CRC_ERRS)     */
    uint32_t spi_errors;
    uint32_t elapsed_ms;
} fpga_selftest_t;

/* Pulse FPGA_RST_N (if wired) and check ID + map version. */
fpga_status_t fpga_init(fpga_link_t *link);

/* Hold FPGA_RST_N low for ~1 ms, then release. No-op if rst_port is NULL. */
void fpga_reset(fpga_link_t *link);

/* Write is fire-and-forget: the FPGA drops frames with a bad CRC and counts
 * them in CRC_ERRS. Use fpga_write_verify for registers that read back. */
fpga_status_t fpga_write(fpga_link_t *link, uint8_t addr, uint32_t value);
fpga_status_t fpga_read(fpga_link_t *link, uint8_t addr, uint32_t *value);
fpga_status_t fpga_write_verify(fpga_link_t *link, uint8_t addr, uint32_t value);

/* Burst access to count (1..FPGA_BURST_MAX) consecutive registers starting at
 * addr, in one frame. Addresses wrap at 0x7F. A burst write is all-or-nothing:
 * the FPGA applies it only if both CRCs match. A burst read retries as a whole
 * on a CRC error, like fpga_read. */
fpga_status_t fpga_write_burst(fpga_link_t *link, uint8_t addr,
                               const uint32_t *values, uint32_t count);
fpga_status_t fpga_read_burst(fpga_link_t *link, uint8_t addr,
                              uint32_t *values, uint32_t count);

/* Read-modify-write helper (not atomic with respect to the FPGA). */
fpga_status_t fpga_modify(fpga_link_t *link, uint8_t addr,
                          uint32_t clear_mask, uint32_t set_mask);

/* Random write/readback stress on SCRATCH. Shows progress on the 7-seg. */
fpga_status_t fpga_selftest(fpga_link_t *link, uint32_t iterations,
                            fpga_selftest_t *result);

/* Same idea with 16-word bursts on SCRATCH_RAM: each iteration is one burst
 * write and one burst read; mismatches count words. */
fpga_status_t fpga_selftest_burst(fpga_link_t *link, uint32_t iterations,
                                  fpga_selftest_t *result);

/* CRC-8, poly 0x07, init 0x00 (exposed for host tests). */
uint8_t fpga_crc8(uint8_t crc, const uint8_t *data, uint32_t len);

const char *fpga_status_str(fpga_status_t s);

#endif /* FPGA_LINK_H */
