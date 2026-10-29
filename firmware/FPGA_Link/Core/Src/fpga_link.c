/*
 * fpga_link.c - STM32 HAL driver for the CRC-8 framed SPI register link.
 * See fpga_link.h and rtl/spi_link.v for the protocol.
 */
#include "fpga_link.h"

#define FPGA_DEFAULT_TIMEOUT_MS 5u

/* ------------------------------------------------------------------------ */
/* CRC-8 (poly 0x07, init 0x00), table-driven                               */
/* ------------------------------------------------------------------------ */
static uint8_t crc_table[256];
static bool    crc_table_ready = false;

static void crc_init_table(void)
{
    for (uint32_t i = 0; i < 256u; i++) {
        uint8_t c = (uint8_t)i;
        for (int b = 0; b < 8; b++)
            c = (c & 0x80u) ? (uint8_t)((c << 1) ^ 0x07u) : (uint8_t)(c << 1);
        crc_table[i] = c;
    }
    crc_table_ready = true;
}

uint8_t fpga_crc8(uint8_t crc, const uint8_t *data, uint32_t len)
{
    if (!crc_table_ready)
        crc_init_table();
    while (len--)
        crc = crc_table[crc ^ *data++];
    return crc;
}

/* ------------------------------------------------------------------------ */
/* Low-level transfer                                                       */
/* ------------------------------------------------------------------------ */
static inline void cs_low(fpga_link_t *l)
{
    HAL_GPIO_WritePin(l->cs_port, l->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(fpga_link_t *l)
{
    HAL_GPIO_WritePin(l->cs_port, l->cs_pin, GPIO_PIN_SET);
}

/* Take the link if it is free. Interrupt-safe: an async start in an ISR and a
 * blocking call in the main loop can't both get it. */
static bool link_acquire(fpga_link_t *l)
{
    uint32_t primask = __get_PRIMASK();
    bool ok;

    __disable_irq();
    ok = !l->busy;
    if (ok)
        l->busy = true;
    __set_PRIMASK(primask);
    return ok;
}

static fpga_status_t xfer(fpga_link_t *l, const uint8_t *tx, uint8_t *rx,
                          uint16_t len)
{
    uint32_t to = l->timeout_ms ? l->timeout_ms : FPGA_DEFAULT_TIMEOUT_MS;
    HAL_StatusTypeDef hs;

    while (!link_acquire(l))
        ;   /* an async frame is in flight; its DMA interrupt frees the link */

    cs_low(l);
    hs = HAL_SPI_TransmitReceive(l->hspi, (uint8_t *)tx, rx, len, to);
    cs_high(l);
    l->busy = false;

    if (hs != HAL_OK) {
        l->spi_errors++;
        return FPGA_ERR_SPI;
    }
    return FPGA_OK;
}

static void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}

/* ------------------------------------------------------------------------ */
/* Register access                                                          */
/* ------------------------------------------------------------------------ */
fpga_status_t fpga_write(fpga_link_t *l, uint8_t addr, uint32_t value)
{
    uint8_t tx[6], rx[6];

    if ((addr & 0x7Fu) == FPGA_BURST_ADDR)   /* the burst escape, not a register */
        return FPGA_ERR_PARAM;

    tx[0] = addr & 0x7Fu;
    tx[1] = (uint8_t)(value >> 24);
    tx[2] = (uint8_t)(value >> 16);
    tx[3] = (uint8_t)(value >> 8);
    tx[4] = (uint8_t)(value);
    tx[5] = fpga_crc8(0, tx, 5);

    l->writes++;
    return xfer(l, tx, rx, sizeof tx);
}

fpga_status_t fpga_read(fpga_link_t *l, uint8_t addr, uint32_t *value)
{
    uint8_t tx[7] = {0}, rx[7];
    fpga_status_t st = FPGA_ERR_CRC;

    if ((addr & 0x7Fu) == FPGA_BURST_ADDR)
        return FPGA_ERR_PARAM;

    tx[0] = 0x80u | (addr & 0x7Fu);
    tx[1] = fpga_crc8(0, tx, 1);

    for (uint32_t attempt = 0; attempt <= l->retries; attempt++) {
        l->reads++;
        st = xfer(l, tx, rx, sizeof tx);
        if (st != FPGA_OK)
            continue;

        if (fpga_crc8(0, &rx[2], 4) == rx[6]) {
            *value = get_be32(&rx[2]);
            return FPGA_OK;
        }
        l->crc_errors++;
        st = FPGA_ERR_CRC;
    }
    return st;
}

/* ------------------------------------------------------------------------ */
/* Burst access (map version 2)                                             */
/*   [0x7F | 0x80 for read] [start addr] [N-1] [CRC(0..2)] data... [CRC]    */
/* ------------------------------------------------------------------------ */
#define BURST_HDR_LEN   4u

/* Build a burst frame in tx; returns its length. values is NULL for a read
 * (the data bytes are then zeros, clocking out the FPGA's reply). */
static uint16_t build_burst(uint8_t *tx, uint8_t addr, const uint32_t *values,
                            uint32_t count)
{
    uint32_t data_len = 4u * count;

    tx[0] = values ? FPGA_BURST_ADDR : (0x80u | FPGA_BURST_ADDR);
    tx[1] = addr & 0x7Fu;
    tx[2] = (uint8_t)(count - 1u);
    tx[3] = fpga_crc8(0, tx, 3);
    for (uint32_t i = 0; i < count; i++)
        put_be32(&tx[BURST_HDR_LEN + 4u * i], values ? values[i] : 0u);
    tx[BURST_HDR_LEN + data_len] = values ? fpga_crc8(0, &tx[BURST_HDR_LEN], data_len) : 0;
    return (uint16_t)(BURST_HDR_LEN + data_len + 1u);
}

/* Check a burst read reply and unpack it */
static bool parse_burst(const uint8_t *rx, uint32_t *values, uint32_t count)
{
    uint32_t data_len = 4u * count;

    if (fpga_crc8(0, &rx[BURST_HDR_LEN], data_len) != rx[BURST_HDR_LEN + data_len])
        return false;
    for (uint32_t i = 0; i < count; i++)
        values[i] = get_be32(&rx[BURST_HDR_LEN + 4u * i]);
    return true;
}

fpga_status_t fpga_write_burst(fpga_link_t *l, uint8_t addr,
                               const uint32_t *values, uint32_t count)
{
    uint8_t tx[FPGA_FRAME_MAX], rx[FPGA_FRAME_MAX];
    uint16_t len;

    if (count == 0u || count > FPGA_BURST_MAX)
        return FPGA_ERR_PARAM;

    len = build_burst(tx, addr, values, count);
    l->writes++;
    return xfer(l, tx, rx, len);
}

fpga_status_t fpga_read_burst(fpga_link_t *l, uint8_t addr,
                              uint32_t *values, uint32_t count)
{
    uint8_t tx[FPGA_FRAME_MAX], rx[FPGA_FRAME_MAX];
    fpga_status_t st = FPGA_ERR_CRC;
    uint16_t len;

    if (count == 0u || count > FPGA_BURST_MAX)
        return FPGA_ERR_PARAM;

    len = build_burst(tx, addr, NULL, count);

    for (uint32_t attempt = 0; attempt <= l->retries; attempt++) {
        l->reads++;
        st = xfer(l, tx, rx, len);
        if (st != FPGA_OK)
            continue;
        if (parse_burst(rx, values, count))
            return FPGA_OK;
        l->crc_errors++;
        st = FPGA_ERR_CRC;
    }
    return st;
}

/* ------------------------------------------------------------------------ */
/* Async bursts (SPI DMA)                                                   */
/* ------------------------------------------------------------------------ */
static fpga_status_t start_async(fpga_link_t *l, uint8_t addr,
                                 const uint32_t *wvalues, uint32_t *rvalues,
                                 uint32_t count, fpga_async_cb_t done)
{
    uint16_t len;

    if (count == 0u || count > FPGA_BURST_MAX)
        return FPGA_ERR_PARAM;
    if (!link_acquire(l))
        return FPGA_ERR_BUSY;

    len = build_burst(l->dma_tx, addr, wvalues, count);
    l->dma_dest  = rvalues;
    l->dma_count = count;
    l->dma_cb    = done;
    if (rvalues) l->reads++; else l->writes++;

    cs_low(l);
    if (HAL_SPI_TransmitReceive_DMA(l->hspi, l->dma_tx, l->dma_rx, len) != HAL_OK) {
        /* If the TX DMA failed to start, HAL returns an error but stays
         * BUSY_TX_RX with the RX DMA armed, refusing every later transfer.
         * Abort to put SPI and both DMA channels back to ready. */
        HAL_SPI_Abort(l->hspi);
        cs_high(l);
        l->busy = false;
        l->spi_errors++;
        return FPGA_ERR_SPI;
    }
    return FPGA_OK;
}

fpga_status_t fpga_read_burst_async(fpga_link_t *l, uint8_t addr,
                                    uint32_t *values, uint32_t count,
                                    fpga_async_cb_t done)
{
    return start_async(l, addr, NULL, values, count, done);
}

fpga_status_t fpga_write_burst_async(fpga_link_t *l, uint8_t addr,
                                     const uint32_t *values, uint32_t count,
                                     fpga_async_cb_t done)
{
    return start_async(l, addr, values, NULL, count, done);
}

void fpga_async_complete(fpga_link_t *l, bool spi_ok)
{
    fpga_status_t st = FPGA_OK;
    fpga_async_cb_t cb = l->dma_cb;

    cs_high(l);
    if (!spi_ok) {
        l->spi_errors++;
        st = FPGA_ERR_SPI;
    } else if (l->dma_dest && !parse_burst(l->dma_rx, l->dma_dest, l->dma_count)) {
        l->crc_errors++;
        st = FPGA_ERR_CRC;
    }
    l->busy = false;   /* before the callback, so it can chain the next frame */
    if (cb)
        cb(l, st);
}

fpga_status_t fpga_write_verify(fpga_link_t *l, uint8_t addr, uint32_t value)
{
    fpga_status_t st = FPGA_ERR_VERIFY;
    uint32_t rb;

    for (uint32_t attempt = 0; attempt <= l->retries; attempt++) {
        st = fpga_write(l, addr, value);
        if (st != FPGA_OK)
            continue;
        st = fpga_read(l, addr, &rb);
        if (st != FPGA_OK)
            continue;
        if (rb == value)
            return FPGA_OK;
        st = FPGA_ERR_VERIFY;
    }
    return st;
}

fpga_status_t fpga_modify(fpga_link_t *l, uint8_t addr,
                          uint32_t clear_mask, uint32_t set_mask)
{
    uint32_t v;
    fpga_status_t st = fpga_read(l, addr, &v);
    if (st != FPGA_OK)
        return st;
    return fpga_write(l, addr, (v & ~clear_mask) | set_mask);
}

/* ------------------------------------------------------------------------ */
/* Init, reset, self-test                                                   */
/* ------------------------------------------------------------------------ */
void fpga_reset(fpga_link_t *l)
{
    if (l->rst_port == NULL)
        return;
    HAL_GPIO_WritePin(l->rst_port, l->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(l->rst_port, l->rst_pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

fpga_status_t fpga_init(fpga_link_t *l)
{
    uint32_t v;
    fpga_status_t st;

    l->reads = l->writes = l->crc_errors = l->spi_errors = 0;
    cs_high(l);
    if (l->rst_port != NULL)
        HAL_GPIO_WritePin(l->rst_port, l->rst_pin, GPIO_PIN_SET);
    fpga_reset(l);

    st = fpga_read(l, FPGA_REG_ID, &v);
    if (st != FPGA_OK)
        return st;
    if (v != FPGA_ID_VALUE)
        return FPGA_ERR_ID;

    st = fpga_read(l, FPGA_REG_VERSION, &v);
    if (st != FPGA_OK)
        return st;
    if (v != FPGA_MAP_VERSION)
        return FPGA_ERR_VERSION;

    return fpga_write(l, FPGA_REG_CTRL, FPGA_CTRL_CLEAR_COUNTERS);
}

static uint32_t xorshift32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return *s = x;
}

/* Common end of both self-tests: timing, error counts and the verdict */
static fpga_status_t selftest_finish(fpga_link_t *l, fpga_selftest_t *r,
                                     uint32_t start, uint32_t base_crc,
                                     uint32_t base_spi)
{
    r->elapsed_ms = HAL_GetTick() - start;
    r->crc_errors = l->crc_errors - base_crc;
    r->spi_errors = l->spi_errors - base_spi;
    r->fpga_crc_errors = 0;
    fpga_read(l, FPGA_REG_CRC_ERRS, &r->fpga_crc_errors);

    if (r->spi_errors)                       return FPGA_ERR_SPI;
    if (r->crc_errors || r->fpga_crc_errors) return FPGA_ERR_CRC;
    if (r->mismatches)                       return FPGA_ERR_VERIFY;
    return FPGA_OK;
}

fpga_status_t fpga_selftest(fpga_link_t *l, uint32_t iterations,
                            fpga_selftest_t *r)
{
    uint32_t seed = 0x1234ABCDu;
    uint32_t base_crc = l->crc_errors;
    uint32_t base_spi = l->spi_errors;
    uint32_t start = HAL_GetTick();
    uint32_t v, rb;
    fpga_status_t st;

    r->iterations = 0;
    r->mismatches = 0;

    /* Retries off for the test so every error is counted */
    uint8_t saved_retries = l->retries;
    l->retries = 0;

    fpga_write(l, FPGA_REG_CTRL, FPGA_CTRL_CLEAR_COUNTERS);

    for (uint32_t i = 0; i < iterations; i++) {
        v = xorshift32(&seed);
        if (fpga_write(l, FPGA_REG_SCRATCH, v) != FPGA_OK)
            continue;
        st = fpga_read(l, FPGA_REG_SCRATCH, &rb);
        if (st == FPGA_OK && rb != v)
            r->mismatches++;
        r->iterations++;

        if ((i & 0x3FFu) == 0)   /* progress on the display */
            fpga_write(l, FPGA_REG_SEG7,
                       FPGA_SEG7_USE_REG | FPGA_SEG7_ALL_DIGITS | ((i >> 10) & 0xFFFFu));
    }

    /* Hand the display back to the frame counter */
    fpga_write(l, FPGA_REG_SEG7, 0);

    l->retries = saved_retries;
    return selftest_finish(l, r, start, base_crc, base_spi);
}

fpga_status_t fpga_selftest_burst(fpga_link_t *l, uint32_t iterations,
                                  fpga_selftest_t *r)
{
    uint32_t seed = 0x89ABCDEFu;
    uint32_t base_crc = l->crc_errors;
    uint32_t base_spi = l->spi_errors;
    uint32_t start = HAL_GetTick();
    uint32_t wr[FPGA_BURST_MAX], rd[FPGA_BURST_MAX];

    r->iterations = 0;
    r->mismatches = 0;

    uint8_t saved_retries = l->retries;
    l->retries = 0;

    fpga_write(l, FPGA_REG_CTRL, FPGA_CTRL_CLEAR_COUNTERS);

    for (uint32_t i = 0; i < iterations; i++) {
        for (uint32_t w = 0; w < FPGA_BURST_MAX; w++)
            wr[w] = xorshift32(&seed);
        if (fpga_write_burst(l, FPGA_REG_SCRATCH_RAM, wr, FPGA_BURST_MAX) != FPGA_OK)
            continue;
        if (fpga_read_burst(l, FPGA_REG_SCRATCH_RAM, rd, FPGA_BURST_MAX) == FPGA_OK)
            for (uint32_t w = 0; w < FPGA_BURST_MAX; w++)
                if (rd[w] != wr[w])
                    r->mismatches++;
        r->iterations++;
    }

    l->retries = saved_retries;
    return selftest_finish(l, r, start, base_crc, base_spi);
}

const char *fpga_status_str(fpga_status_t s)
{
    switch (s) {
    case FPGA_OK:          return "OK";
    case FPGA_ERR_SPI:     return "SPI error/timeout";
    case FPGA_ERR_CRC:     return "CRC error";
    case FPGA_ERR_VERIFY:  return "readback mismatch";
    case FPGA_ERR_ID:      return "bad ID (no bitstream, or wiring)";
    case FPGA_ERR_VERSION: return "register map version mismatch";
    case FPGA_ERR_PARAM:   return "bad argument";
    case FPGA_ERR_BUSY:    return "link busy";
    default:               return "unknown";
    }
}
