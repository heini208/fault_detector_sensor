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

The BMM150 dependency is pinned to a current upstream revision. The older
PlatformIO registry release (`1.0.0`) deadlocks inside its I2C read implementation
with the ESP32 Arduino Wire mutex and must not be substituted.

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
   baud, local echo, send-on-enter input, and released ESP32 RTS/DTR reset lines.

Some ESP32 boards require holding the `BOOT` button when upload starts and
releasing it when the terminal displays `Connecting...`.

On Linux, a permission error for `/dev/ttyUSB*` or `/dev/ttyACM*` usually means
the user needs serial-port group access. On Ubuntu this is commonly the
`dialout` group; log out and back in after an administrator adds the user.

## Hardware bring-up

The firmware starts in `IDLE`, so it does not emit measurement samples until
requested. The command parser accepts LF, CR, and CRLF line endings. Local echo
is enabled, and input is buffered until Enter is pressed. Each command also
produces an `OK` or `ERROR` response. Send:

```text
status
calibrate
start
stop
help
```

During `calibrate`, move the complete sensor mount through a figure-eight for
10 seconds. Calibration is currently held in RAM and is cleared by rebooting.
While recording, compensated BMM150 values are displayed at 10 Hz. Samples are
not printed over serial so they cannot interfere with interactive commands. The
later micro-ROS integration will publish timestamped samples independently from
this diagnostic command interface.

## Run host-side state-machine tests

From the PlatformIO terminal:

```bash
pio test -e native
```

The native tests require a local C++ compiler and do not require an ESP32.
