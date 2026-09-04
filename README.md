# fault_detector_sensor

ESP32 firmware for the hand-mounted BMM150 inspection sensor.

The firmware initializes the BMM150 and ST7735 display, boots in `IDLE`, accepts
acknowledged acquisition commands over the serial monitor, and maintains a
rtmicro-ROS UDP connection over Wi-Fi. While `RECORDING`, it publishes timestamped
magnetic-field measurements. The acknowledged ROS acquisition service is the
next integration step.

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
   `/home/marcel/Projects/spot/spot_sensor/fault_detector_sensor`.
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
While recording, compensated BMM150 values are displayed and published at
10 Hz. Samples are not printed over serial so they cannot interfere with
interactive commands.

## Configure Wi-Fi and the micro-ROS Agent

Wi-Fi credentials and the Agent endpoint are stored in ESP32 nonvolatile
storage and are not committed to this repository. The default Agent endpoint is
`192.168.178.69:8888`. On first boot, use the serial monitor to enter:

~~~text
set-wifi <ssid> <password>
show
~~~

For an SSID containing spaces, surround the SSID with double quotes:

~~~text
set-wifi "SSID with spaces" <password>
~~~

The password may contain spaces. Change the Agent endpoint when necessary with:

~~~text
set-agent <IPv4-address> <port>
~~~

Erase the stored Wi-Fi credentials and restore the default Agent endpoint with:

~~~text
reset-network
~~~

The password is never printed back by `show`. The firmware reports Wi-Fi and
Agent connection state changes without emitting a periodic heartbeat. Run the
host Agent with:

~~~bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v 6
~~~

The PlatformIO environment builds micro-ROS for ROS 2 Humble with its Wi-Fi
transport. The first firmware build takes longer because PlatformIO builds the
micro-ROS library locally.

Once the serial monitor reports `ROS state=READY`, the node publishes
`sensor_msgs/msg/MagneticField` on
`/sensors/bmm150_probe/magnetic_field`. Values are converted from microtesla to
Tesla, timestamps are synchronized from the Agent, the frame is
`bmm150_probe_probe`, and sensor-data best-effort QoS is used. Publication only
occurs in `RECORDING`; use the serial `start` and `stop` commands until the ROS
acquisition service is implemented.

Inspect the stream from a ROS 2 terminal with:

~~~bash
ros2 topic echo /sensors/bmm150_probe/magnetic_field \
  sensor_msgs/msg/MagneticField --qos-reliability best_effort
~~~

## Run host-side state-machine tests

From the PlatformIO terminal:

```bash
pio test -e native
```

The native tests require a local C++ compiler and do not require an ESP32.
