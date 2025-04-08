# SDIO_RP2350: High performance SD-card access on RP2350 microcontroller

This library implements 4-bit SDIO mode access to SD, MicroSD and eMMC memories.
It runs on RP2350 microcontroller and takes advantage of the PIO hardware to accelerate access.

The library is divider to low-level SDIO communication in `sdio_rp2350.cpp`, and a high-level driver
`sdfat_sdcard_rp2350.cpp` that interfaces with the [SdFat](https://github.com/greiman/SdFat) filesystem library.

## Performance

The code supports 3.3V access modes, where the highest specified SD card access mode is 50 MHz.
This results in data transfer rates close to 25 MB/s. Some overhead occurs due to request initiation
and checksum calculation, but for multi-sector burst transfers close to 25 MB/s read speed can be achieved.

There is additional support for `SDIO_HIGHSPEED_OVERCLOCK` mode where SDIO clock rate is increased to 75 MHz.
This can achieve up to 37 MB/s transfer speeds. Based on testing, it works on practically every card that
supports the normal 50 MHz high-speed mode.

The `sdfat_sdcard_rp2350.cpp` driver implements automatic communication check and reduces the clock
rate if needed. Early 2000s MMC cards do not support clock rates over 20 MHz.

## History

This library has been developed by Rabbit Hole Computing™ for use in its [ZuluSCSI firmware](https://github.com/ZuluSCSI/ZuluSCSI-firmware). It is a redesigned version of our earlier RP2040 SDIO library.

## License

The library is [licensed under MIT license](LICENSE.md).
