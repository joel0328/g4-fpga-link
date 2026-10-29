# FPGA_Link firmware (STM32CubeIDE project)

Nucleo-G431KB side of the SPI register link. See the top-level `README.md` for
bring-up.

- CubeIDE: File > Import > General > Existing Projects into Workspace > this folder.
- Command line: `make` (needs arm-none-eabi-gcc), output in `build/`.
- `FPGA_Link.ioc` opens in CubeMX; Generate Code keeps everything inside USER CODE blocks.
