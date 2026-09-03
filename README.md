# fault_detector_sensor

ESP32 firmware for the hand-mounted BMM150 inspection sensor.

The current firmware is a hardware bring-up foundation. It initializes the
BMM150 and ST7735 display, boots in `IDLE`, and accepts acknowledged acquisition
commands over the serial monitor. micro-ROS transport and the ROS acquisition
service will be added after this local control path is validated on hardware.

## Current hardware configuration

The default PlatformIO environment targets a classic Espressif ESP32 Dev Module
(`esp32dev`). The display wiring matches the existing prototype:

| Signal | ESP32 GPIO |
| --- | ---: |
| TFT CS | 5 |
| TFT RST | 16 |
| TFT DC | 19 |
| TFT MOSI | 23 |
| TFT SCLK | 18 |

The Seeed Studio BMM150 library uses the ESP32's default I2C pins (SDA 21 and
SCL 22 on a classic ESP32 Dev Module). Change `platformio.ini` if the installed
board is not compatible with `esp32dev`, and change `include/hardware_config.hpp`
if the wiring differs.

## Flash from VS Code

1. Install the `PlatformIO IDE` extension in VS Code.
2. Open this repository folder, not only `src/main.cpp`:
   `/home/marcel/Projects/fault_detector_sensor`.
3. Connect the ESP32 over a data-capable USB cable.
4. Wait for PlatformIO to install the Espressif platform and declared libraries.
5. Click the checkmark in the VS Code status bar to build.
6. Click the right-arrow in the status bar to upload. Alternatively, open the
   PlatformIO sidebar and run `esp32dev > General > Upload`.
7. If automatic port detection fails, run `PlatformIO: Upload` from the Command
   Palette and select the ESP32 serial device. A fixed `upload_port` can also be
   added locally to `platformio.ini`, for example `/dev/ttyUSB0`.
8. Open the plug icon (PlatformIO Serial Monitor). It is configured for 115200
   baud.

Some ESP32 boards require holding the `BOOT` button when upload starts and
releasing it when the terminal displays `Connecting...`.

On Linux, a permission error for `/dev/ttyUSB*` or `/dev/ttyACM*` usually means
the user needs serial-port group access. On Ubuntu this is commonly the
`dialout` group; log out and back in after an administrator adds the user.

## Hardware bring-up

The firmware starts in `IDLE`, so it does not emit measurement samples until
requested. The command parser accepts LF, CR, and CRLF line endings. Send:

```text
status
calibrate
start
stop
help
```

During `calibrate`, move the complete sensor mount through a figure-eight for
10 seconds. Calibration is currently held in RAM and is cleared by rebooting.
While recording, compensated BMM150 values are displayed and printed at 10 Hz:

```text
sample uptime_us=1234567 x_uT=12 y_uT=-4 z_uT=31
```

`uptime_us` is only a local bring-up timestamp. The later micro-ROS integration
must synchronize the ESP32 clock before populating a ROS source timestamp.

## Run host-side state-machine tests

From the PlatformIO terminal:

```bash
pio test -e native
```

The native tests require a local C++ compiler and do not require an ESP32.
