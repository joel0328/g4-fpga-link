/*
 * fpga_link_app.h - bring-up application for the Nucleo-G431KB <-> Basys 3 link.
 */
#ifndef FPGA_LINK_APP_H
#define FPGA_LINK_APP_H

#include <stdbool.h>

/* Call once after the MX_*_Init() calls. Prints to the ST-LINK VCP. */
void fpga_app_init(void);

/* Call from the main loop. Non-blocking apart from SPI transfers. */
void fpga_app_poll(void);

/* True once the FPGA answered with the right ID and map version. */
bool fpga_app_link_up(void);

#endif /* FPGA_LINK_APP_H */
