/*
 * host_test.c - checks the STM32 driver against the RTL without hardware.
 *
 *   ./host_test record frames.hex   writes the MOSI bytes the driver sends
 *                                   for a fixed transaction script
 *   (tb_replay.v plays frames.hex into the RTL and writes miso.hex)
 *   ./host_test check miso.hex      feeds the FPGA's MISO bytes back through
 *                                   the driver and checks every result
 *
 * Also checks the CRC-8 against the standard check value.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fpga_link.h"

/* ---------------- HAL stub ---------------- */
static FILE *rec_file;          /* record mode: MOSI bytes out          */
static uint8_t miso_buf[4096];  /* check mode: MISO bytes in            */
static size_t  miso_len, miso_pos;
static int     cs_low_count;

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx,
                                          uint8_t *rx, uint16_t len,
                                          uint32_t timeout)
{
    (void)h; (void)timeout;
    if (rec_file) {
        fprintf(rec_file, "%02X", len);   /* frame length, then bytes */
        for (uint16_t i = 0; i < len; i++)
            fprintf(rec_file, " %02X", tx[i]);
        fprintf(rec_file, "\n");
        memset(rx, 0, len);
        return HAL_OK;
    }
    if (miso_pos + len > miso_len)
        return HAL_TIMEOUT;
    memcpy(rx, &miso_buf[miso_pos], len);
    miso_pos += len;
    return HAL_OK;
}

/* DMA variant: same bytes; the script then calls fpga_async_complete()
 * itself, standing in for the DMA-complete interrupt. dma_fail_next makes
 * the next start fail the way HAL does when the TX DMA channel is busy. */
static int dma_fail_next, abort_calls;

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h, uint8_t *tx,
                                              uint8_t *rx, uint16_t len)
{
    if (dma_fail_next) {
        dma_fail_next = 0;
        return HAL_ERROR;
    }
    return HAL_SPI_TransmitReceive(h, tx, rx, len, 0);
}

HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h)
{
    (void)h;
    abort_calls++;
    return HAL_OK;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s)
{
    (void)p; (void)pin;
    if (s == GPIO_PIN_RESET) cs_low_count++;
}
void     HAL_Delay(uint32_t ms) { (void)ms; }
uint32_t HAL_GetTick(void) { return 0; }

/* ---------------- transaction script ---------------- */
static int failures;
#define EXPECT(cond, ...) do { if (!(cond)) { failures++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

static SPI_HandleTypeDef hspi;
static GPIO_TypeDef cs_port;

static int           async_calls;
static fpga_status_t async_status;
static void async_done(fpga_link_t *link, fpga_status_t st)
{
    (void)link;
    async_calls++;
    async_status = st;
}

static void script(int checking)
{
    static fpga_link_t l;   /* static: it holds the DMA buffers */
    l = (fpga_link_t){ .hspi = &hspi, .cs_port = &cs_port, .cs_pin = 1,
                       .rst_port = NULL, .retries = 0 };
    uint32_t v = 0;
    fpga_status_t st;

    /* 1. ID and version */
    st = fpga_read(&l, FPGA_REG_ID, &v);
    if (checking) EXPECT(st == FPGA_OK && v == FPGA_ID_VALUE, "ID read: st=%d v=%08X", st, v);
    st = fpga_read(&l, FPGA_REG_VERSION, &v);
    if (checking) EXPECT(st == FPGA_OK && v == FPGA_MAP_VERSION, "VERSION read: st=%d v=%08X", st, v);

    /* 2. Scratch patterns */
    const uint32_t pats[] = { 0xCAFEF00Du, 0x00000000u, 0xFFFFFFFFu, 0x80000001u, 0x5A5AA5A5u };
    for (size_t i = 0; i < sizeof pats / sizeof pats[0]; i++) {
        fpga_write(&l, FPGA_REG_SCRATCH, pats[i]);
        st = fpga_read(&l, FPGA_REG_SCRATCH, &v);
        if (checking) EXPECT(st == FPGA_OK && v == pats[i], "SCRATCH %08X: st=%d v=%08X", pats[i], st, v);
    }

    /* 3. LEDs */
    fpga_write(&l, FPGA_REG_LEDS, 0x0000A5C3u);
    st = fpga_read(&l, FPGA_REG_LEDS, &v);
    if (checking) EXPECT(st == FPGA_OK && v == 0xA5C3u, "LEDS: st=%d v=%08X", st, v);

    /* 4. Unmapped (0x7F is the burst escape, so 0x7E) */
    st = fpga_read(&l, 0x7E, &v);
    if (checking) EXPECT(st == FPGA_OK && v == FPGA_UNMAPPED_VALUE, "unmapped: st=%d v=%08X", st, v);

    /* 5. Frame counter: 2 + 10 + 2 + 1 = 15 good frames so far, read latched
          before this frame counts itself */
    st = fpga_read(&l, FPGA_REG_FRAMES_OK, &v);
    if (checking) EXPECT(st == FPGA_OK && v == 15u, "FRAMES_OK: st=%d v=%lu (expected 15)", st, (unsigned long)v);

    st = fpga_read(&l, FPGA_REG_CRC_ERRS, &v);
    if (checking) EXPECT(st == FPGA_OK && v == 0u, "CRC_ERRS: st=%d v=%lu", st, (unsigned long)v);

    /* 6. Bursts: 16 words through SCRATCH_RAM, ID + VERSION in one frame */
    uint32_t wr[FPGA_BURST_MAX], rd[FPGA_BURST_MAX] = {0};
    for (uint32_t i = 0; i < FPGA_BURST_MAX; i++)
        wr[i] = (0x01020304u * (i + 1u)) ^ 0xA5A5A5A5u;
    st = fpga_write_burst(&l, FPGA_REG_SCRATCH_RAM, wr, FPGA_BURST_MAX);
    if (checking) EXPECT(st == FPGA_OK, "burst write: st=%d", st);
    st = fpga_read_burst(&l, FPGA_REG_SCRATCH_RAM, rd, FPGA_BURST_MAX);
    if (checking) EXPECT(st == FPGA_OK && memcmp(rd, wr, sizeof wr) == 0,
                         "burst read x16: st=%d rd[0]=%08X rd[15]=%08X", st, rd[0], rd[15]);
    st = fpga_read(&l, FPGA_REG_SCRATCH_RAM + 15u, &v);
    if (checking) EXPECT(st == FPGA_OK && v == wr[15], "RAM[15] single read: st=%d v=%08X", st, v);
    st = fpga_read_burst(&l, FPGA_REG_ID, rd, 2);
    if (checking) EXPECT(st == FPGA_OK && rd[0] == FPGA_ID_VALUE && rd[1] == FPGA_MAP_VERSION,
                         "burst ID+VERSION: st=%d %08X %08X", st, rd[0], rd[1]);

    /* 7. Bad arguments are refused without sending a frame */
    st = fpga_read_burst(&l, FPGA_REG_SCRATCH_RAM, rd, 0);
    if (checking) EXPECT(st == FPGA_ERR_PARAM, "burst count 0: st=%d", st);
    st = fpga_write_burst(&l, FPGA_REG_SCRATCH_RAM, wr, FPGA_BURST_MAX + 1u);
    if (checking) EXPECT(st == FPGA_ERR_PARAM, "burst count 17: st=%d", st);
    st = fpga_read(&l, FPGA_BURST_ADDR, &v);
    if (checking) EXPECT(st == FPGA_ERR_PARAM, "single read of 0x7F: st=%d", st);
    st = fpga_write(&l, FPGA_BURST_ADDR, 0);
    if (checking) EXPECT(st == FPGA_ERR_PARAM, "single write of 0x7F: st=%d", st);

    /* 8. Async bursts, completed by hand as the DMA interrupt would */
    for (uint32_t i = 0; i < 4; i++)
        wr[i] = 0xC0DE0000u + i;
    async_calls = 0;
    st = fpga_write_burst_async(&l, FPGA_REG_SCRATCH_RAM + 4u, wr, 4, async_done);
    if (checking) EXPECT(st == FPGA_OK, "async write start: st=%d", st);
    st = fpga_read_burst_async(&l, FPGA_REG_SCRATCH_RAM, rd, 1, async_done);
    if (checking) EXPECT(st == FPGA_ERR_BUSY, "async start while busy: st=%d", st);
    fpga_async_complete(&l, true);
    if (checking) EXPECT(async_calls == 1 && async_status == FPGA_OK,
                         "async write done: calls=%d st=%d", async_calls, async_status);

    memset(rd, 0, sizeof rd);
    st = fpga_read_burst_async(&l, FPGA_REG_SCRATCH_RAM + 3u, rd, 6, async_done);
    if (checking) EXPECT(st == FPGA_OK, "async read start: st=%d", st);
    fpga_async_complete(&l, true);
    /* RAM[3] = wr[3] from step 6, RAM[4..7] = C0DE0000.., RAM[8] = step-6 value */
    if (checking) EXPECT(async_calls == 2 && async_status == FPGA_OK &&
                         rd[0] == ((0x01020304u * 4u) ^ 0xA5A5A5A5u) &&
                         rd[1] == 0xC0DE0000u && rd[4] == 0xC0DE0003u &&
                         rd[5] == ((0x01020304u * 9u) ^ 0xA5A5A5A5u),
                         "async read: calls=%d st=%d rd=%08X %08X %08X %08X",
                         async_calls, async_status, rd[0], rd[1], rd[4], rd[5]);

    /* An SPI error reported by the interrupt reaches the callback */
    st = fpga_read_burst_async(&l, FPGA_REG_ID, rd, 1, async_done);
    fpga_async_complete(&l, false);
    if (checking) EXPECT(st == FPGA_OK && async_calls == 3 && async_status == FPGA_ERR_SPI,
                         "async SPI error: calls=%d st=%d", async_calls, async_status);

    /* A failed DMA start aborts the SPI, frees the link and sends nothing */
    abort_calls = 0;
    dma_fail_next = 1;
    st = fpga_write_burst_async(&l, FPGA_REG_SCRATCH_RAM, wr, 1, async_done);
    if (checking) EXPECT(st == FPGA_ERR_SPI && abort_calls == 1 && !l.busy && async_calls == 3,
                         "failed DMA start: st=%d aborts=%d busy=%d calls=%d",
                         st, abort_calls, (int)l.busy, async_calls);

    /* The link is free again for blocking calls */
    st = fpga_read(&l, FPGA_REG_ID, &v);
    if (checking) EXPECT(st == FPGA_OK && v == FPGA_ID_VALUE, "read after async: st=%d v=%08X", st, v);
}

static int load_miso(const char *path)
{
    FILE *f = fopen(path, "r");
    unsigned b;
    if (!f) { perror(path); return -1; }
    while (fscanf(f, "%x", &b) == 1 && miso_len < sizeof miso_buf)
        miso_buf[miso_len++] = (uint8_t)b;
    fclose(f);
    return 0;
}

int main(int argc, char **argv)
{
    /* CRC-8/SMBUS check value */
    const uint8_t chk[] = "123456789";
    uint8_t c = fpga_crc8(0, chk, 9);
    EXPECT(c == 0xF4, "CRC-8 check value %02X != F4", c);

    /* Known frame: write SCRATCH = CAFEF00D -> header 02 */
    uint8_t fr[5] = { 0x02, 0xCA, 0xFE, 0xF0, 0x0D };
    printf("CRC of write(0x02, 0xCAFEF00D) frame: %02X\n", fpga_crc8(0, fr, 5));

    if (argc == 3 && strcmp(argv[1], "record") == 0) {
        rec_file = fopen(argv[2], "w");
        if (!rec_file) { perror(argv[2]); return 2; }
        script(0);
        fclose(rec_file);
        printf("recorded %d frames to %s\n", cs_low_count, argv[2]);
    } else if (argc == 3 && strcmp(argv[1], "check") == 0) {
        if (load_miso(argv[2])) return 2;
        script(1);
        EXPECT(miso_pos == miso_len, "consumed %zu of %zu MISO bytes", miso_pos, miso_len);
        printf("checked %zu MISO bytes from the RTL\n", miso_len);
    }

    printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
    return failures ? 1 : 0;
}
