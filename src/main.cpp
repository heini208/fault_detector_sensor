#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <bmm150.h>
#include <bmm150_defs.h>

#include <cstdint>
#include <cstring>

#include "acquisition_state_machine.hpp"
#include "hardware_config.hpp"
#include "line_command_buffer.hpp"

namespace {

using fault_detector_sensor::AcquisitionStateMachine;
using fault_detector_sensor::CommandFeedResult;
using fault_detector_sensor::LineCommandBuffer;
using fault_detector_sensor::ToString;
namespace hardware = fault_detector_sensor::hardware;

struct MagneticFieldSample {
  std::int16_t x_microtesla;
  std::int16_t y_microtesla;
  std::int16_t z_microtesla;
  std::uint32_t uptime_us;
};

struct MagneticFieldOffset {
  std::int16_t x_microtesla{0};
  std::int16_t y_microtesla{0};
  std::int16_t z_microtesla{0};
};

Adafruit_ST7735 display(hardware::kTftChipSelectPin,
                        hardware::kTftDataCommandPin, hardware::kTftMosiPin,
                        hardware::kTftClockPin, hardware::kTftResetPin);
BMM150 magnetometer;
AcquisitionStateMachine acquisition;
MagneticFieldOffset offset;
bool magnetometer_ready = false;
std::uint32_t last_sample_ms = 0;

LineCommandBuffer command_buffer;

MagneticFieldSample ReadMagneticField() {
  magnetometer.read_mag_data();
  return {
      static_cast<std::int16_t>(magnetometer.mag_data.x - offset.x_microtesla),
      static_cast<std::int16_t>(magnetometer.mag_data.y - offset.y_microtesla),
      static_cast<std::int16_t>(magnetometer.mag_data.z - offset.z_microtesla),
      micros(),
  };
}

void DrawState() {
  display.fillRect(0, 0, 160, 18, ST77XX_BLACK);
  display.setCursor(0, 0);
  display.setTextColor(acquisition.is_recording() ? ST77XX_GREEN
                                                  : ST77XX_YELLOW);
  display.setTextSize(2);
  display.print(ToString(acquisition.state()));
}

void DrawSample(const MagneticFieldSample &sample) {
  display.setTextWrap(false);
  display.setTextColor(ST77XX_RED);
  display.setTextSize(2);

  display.fillRect(0, 30, 160, 54, ST77XX_BLACK);
  display.setCursor(0, 30);
  display.print("X: ");
  display.println(sample.x_microtesla);
  display.print("Y: ");
  display.println(sample.y_microtesla);
  display.print("Z: ");
  display.println(sample.z_microtesla);
}

void PrintSample(const MagneticFieldSample &sample) {
  Serial.print("sample uptime_us=");
  Serial.print(sample.uptime_us);
  Serial.print(" x_uT=");
  Serial.print(sample.x_microtesla);
  Serial.print(" y_uT=");
  Serial.print(sample.y_microtesla);
  Serial.print(" z_uT=");
  Serial.println(sample.z_microtesla);
}

void PrintAcknowledgement(bool success, const char *detail) {
  Serial.print(success ? "OK " : "ERROR ");
  Serial.print("state=");
  Serial.print(ToString(acquisition.state()));
  Serial.print(" detail=");
  Serial.println(detail);
}

void SetRecording(bool enabled) {
  if (enabled && !magnetometer_ready) {
    PrintAcknowledgement(false, "BMM150 unavailable");
    return;
  }

  const auto result = acquisition.set_recording(enabled);
  if (result.changed) {
    magnetometer.set_op_mode(enabled ? BMM150_NORMAL_MODE : BMM150_SLEEP_MODE);
    if (enabled) {
      last_sample_ms = millis() - hardware::kSamplePeriodMs;
    }
    DrawState();
  }
  PrintAcknowledgement(result.success, result.detail);
}

void Calibrate() {
  if (!magnetometer_ready) {
    PrintAcknowledgement(false, "BMM150 unavailable");
    return;
  }
  if (acquisition.is_recording()) {
    PrintAcknowledgement(false, "stop recording before calibration");
    return;
  }

  Serial.println(
      "CALIBRATING Move the sensor in a figure eight for 10 seconds");
  display.fillScreen(ST77XX_BLACK);
  display.setCursor(0, 0);
  display.setTextColor(ST77XX_CYAN);
  display.setTextSize(2);
  display.println("CALIBRATING");
  display.setTextSize(1);
  display.println("Move in figure 8");

  magnetometer.set_op_mode(BMM150_NORMAL_MODE);
  delay(100);
  magnetometer.read_mag_data();

  std::int16_t x_min = magnetometer.mag_data.x;
  std::int16_t x_max = magnetometer.mag_data.x;
  std::int16_t y_min = magnetometer.mag_data.y;
  std::int16_t y_max = magnetometer.mag_data.y;
  std::int16_t z_min = magnetometer.mag_data.z;
  std::int16_t z_max = magnetometer.mag_data.z;
  const std::uint32_t started_at_ms = millis();

  while (millis() - started_at_ms < hardware::kCalibrationDurationMs) {
    magnetometer.read_mag_data();
    x_min = min(x_min, magnetometer.mag_data.x);
    x_max = max(x_max, magnetometer.mag_data.x);
    y_min = min(y_min, magnetometer.mag_data.y);
    y_max = max(y_max, magnetometer.mag_data.y);
    z_min = min(z_min, magnetometer.mag_data.z);
    z_max = max(z_max, magnetometer.mag_data.z);
    Serial.print('.');
    delay(hardware::kSamplePeriodMs);
  }

  offset.x_microtesla = static_cast<std::int16_t>(x_min + (x_max - x_min) / 2);
  offset.y_microtesla = static_cast<std::int16_t>(y_min + (y_max - y_min) / 2);
  offset.z_microtesla = static_cast<std::int16_t>(z_min + (z_max - z_min) / 2);
  magnetometer.set_op_mode(BMM150_SLEEP_MODE);

  Serial.println();
  display.fillScreen(ST77XX_BLACK);
  DrawState();
  PrintAcknowledgement(true, "calibration complete");
}

void PrintHelp() {
  Serial.println("Commands: start, stop, status, calibrate, help");
}

void ProcessCommand(const char *command) {
  if (strcmp(command, "start") == 0) {
    SetRecording(true);
  } else if (strcmp(command, "stop") == 0) {
    SetRecording(false);
  } else if (strcmp(command, "status") == 0) {
    PrintAcknowledgement(magnetometer_ready, magnetometer_ready
                                                 ? "BMM150 ready"
                                                 : "BMM150 unavailable");
  } else if (strcmp(command, "calibrate") == 0) {
    Calibrate();
  } else if (strcmp(command, "help") == 0) {
    PrintHelp();
  } else if (command[0] != '\0') {
    PrintAcknowledgement(false, "unknown command");
  }
}

void PollSerialCommands() {
  while (Serial.available() > 0) {
    const char input = static_cast<char>(Serial.read());
    const auto result = command_buffer.Feed(input);
    if (result == CommandFeedResult::kCommandReady) {
      ProcessCommand(command_buffer.command());
      command_buffer.Reset();
    } else if (result == CommandFeedResult::kOverflow) {
      PrintAcknowledgement(false, "command too long");
    }
  }
}

void InitializeDisplay() {
  display.initR(INITR_GREENTAB);
  display.setSPISpeed(40000000);
  display.setRotation(1);
  display.fillScreen(ST77XX_BLACK);
  DrawState();
}

} // namespace

void setup() {
  Serial.begin(hardware::kSerialBaud);
  delay(250);
  Serial.println();
  Serial.println("BOOT fault_detector_sensor");

  InitializeDisplay();
  Serial.println("Display initialized");

  Serial.println("Initializing BMM150");
  magnetometer_ready = magnetometer.initialize() == BMM150_OK;
  if (magnetometer_ready) {
    magnetometer.set_op_mode(BMM150_SLEEP_MODE);
    Serial.println("BMM150 initialized");
  } else {
    Serial.println("ERROR BMM150 chip ID could not be read");
  }

  Serial.println("fault_detector_sensor ready in IDLE");
  PrintHelp();
}

void loop() {
  PollSerialCommands();

  if (!acquisition.is_recording()) {
    delay(1);
    return;
  }

  const std::uint32_t now_ms = millis();
  if (now_ms - last_sample_ms < hardware::kSamplePeriodMs) {
    return;
  }

  last_sample_ms = now_ms;
  const auto sample = ReadMagneticField();
  PrintSample(sample);
  DrawSample(sample);
}
