#include "micro_ros_connection.hpp"

#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

namespace fault_detector_sensor {
namespace {

constexpr std::uint32_t kWifiRetryPeriodMs = 10000;
constexpr std::uint32_t kAgentPingPeriodMs = 5000;
constexpr int kAgentPingTimeoutMs = 200;
constexpr std::uint8_t kAgentPingAttempts = 3;
constexpr std::uint8_t kAgentDisconnectThreshold = 3;

bool DeadlineReached(std::uint32_t now, std::uint32_t deadline) {
  return static_cast<std::int32_t>(now - deadline) >= 0;
}

} // namespace

void MicroRosConnection::Begin(
    const NetworkConfiguration &configuration) {
  configuration_ = configuration;
  transport_configured_ = false;
  agent_checked_ = false;
  consecutive_agent_failures_ = 0;

  if (!configuration_.has_wifi_configuration() ||
      !configuration_.has_agent_configuration()) {
    SetState(MicroRosConnectionState::kUnconfigured);
    Serial.println(
        "NETWORK state=UNCONFIGURED detail=run set-wifi then show");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  StartWifiAttempt();
}

void MicroRosConnection::Reconfigure(
    const NetworkConfiguration &configuration) {
  WiFi.disconnect();
  Begin(configuration);
}

void MicroRosConnection::Spin() {
  if (state_ == MicroRosConnectionState::kUnconfigured) {
    return;
  }

  const std::uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED) {
    if (state_ != MicroRosConnectionState::kWifiConnecting) {
      transport_configured_ = false;
      agent_checked_ = false;
      consecutive_agent_failures_ = 0;
      SetState(MicroRosConnectionState::kWifiConnecting);
      Serial.println("NETWORK state=WIFI_CONNECTING detail=connection lost");
    }

    if (DeadlineReached(now, next_wifi_attempt_ms_)) {
      WiFi.disconnect();
      WiFi.begin(configuration_.wifi_ssid.c_str(),
                 configuration_.wifi_password.c_str());
      next_wifi_attempt_ms_ = now + kWifiRetryPeriodMs;
    }
    return;
  }

  if (!transport_configured_) {
    Serial.print("NETWORK state=WIFI_CONNECTED ip=");
    Serial.println(WiFi.localIP());
    ConfigureTransport();
  }

  if (!transport_configured_ ||
      !DeadlineReached(now, next_agent_ping_ms_)) {
    return;
  }
  next_agent_ping_ms_ = now + kAgentPingPeriodMs;

  const bool agent_available =
      rmw_uros_ping_agent(kAgentPingTimeoutMs, kAgentPingAttempts) ==
      RMW_RET_OK;
  MicroRosConnectionState next_state;
  if (agent_available) {
    consecutive_agent_failures_ = 0;
    next_state = MicroRosConnectionState::kAgentConnected;
  } else {
    if (consecutive_agent_failures_ < kAgentDisconnectThreshold) {
      ++consecutive_agent_failures_;
    }
    if (state_ == MicroRosConnectionState::kAgentConnected &&
        consecutive_agent_failures_ < kAgentDisconnectThreshold) {
      agent_checked_ = true;
      return;
    }
    next_state = MicroRosConnectionState::kAgentUnavailable;
  }
  if (!agent_checked_ || next_state != state_) {
    SetState(next_state);
    Serial.print("NETWORK state=");
    Serial.print(ToString(state_));
    Serial.print(" endpoint=");
    Serial.print(configuration_.agent_address);
    Serial.print(':');
    Serial.println(configuration_.agent_port);
  }
  agent_checked_ = true;
}

MicroRosConnectionState MicroRosConnection::state() const { return state_; }

IPAddress MicroRosConnection::local_ip() const { return WiFi.localIP(); }

void MicroRosConnection::StartWifiAttempt() {
  SetState(MicroRosConnectionState::kWifiConnecting);
  Serial.print("NETWORK state=WIFI_CONNECTING ssid=");
  Serial.println(configuration_.wifi_ssid);
  WiFi.begin(configuration_.wifi_ssid.c_str(),
             configuration_.wifi_password.c_str());
  next_wifi_attempt_ms_ = millis() + kWifiRetryPeriodMs;
}

void MicroRosConnection::ConfigureTransport() {
  IPAddress agent_ip;
  if (!agent_ip.fromString(configuration_.agent_address)) {
    SetState(MicroRosConnectionState::kUnconfigured);
    Serial.println("NETWORK state=UNCONFIGURED detail=invalid agent address");
    return;
  }

  set_microros_wifi_transports(
      const_cast<char *>(configuration_.wifi_ssid.c_str()),
      const_cast<char *>(configuration_.wifi_password.c_str()), agent_ip,
      configuration_.agent_port);
  transport_configured_ = true;
  next_agent_ping_ms_ = millis();
}

void MicroRosConnection::SetState(MicroRosConnectionState state) {
  state_ = state;
}

const char *ToString(MicroRosConnectionState state) {
  switch (state) {
  case MicroRosConnectionState::kUnconfigured:
    return "UNCONFIGURED";
  case MicroRosConnectionState::kWifiConnecting:
    return "WIFI_CONNECTING";
  case MicroRosConnectionState::kAgentUnavailable:
    return "AGENT_UNAVAILABLE";
  case MicroRosConnectionState::kAgentConnected:
    return "AGENT_CONNECTED";
  }
  return "UNKNOWN";
}

} // namespace fault_detector_sensor
