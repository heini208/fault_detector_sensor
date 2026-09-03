#pragma once

#include <Arduino.h>

#include "network_configuration.hpp"

namespace fault_detector_sensor {

enum class MicroRosConnectionState {
  kUnconfigured,
  kWifiConnecting,
  kAgentUnavailable,
  kAgentConnected,
};

class MicroRosConnection {
public:
  void Begin(const NetworkConfiguration &configuration);
  void Reconfigure(const NetworkConfiguration &configuration);
  void Spin();

  MicroRosConnectionState state() const;
  IPAddress local_ip() const;

private:
  void StartWifiAttempt();
  void ConfigureTransport();
  void SetState(MicroRosConnectionState state);

  NetworkConfiguration configuration_;
  MicroRosConnectionState state_{MicroRosConnectionState::kUnconfigured};
  bool transport_configured_{false};
  bool agent_checked_{false};
  std::uint8_t consecutive_agent_failures_{0};
  std::uint32_t next_wifi_attempt_ms_{0};
  std::uint32_t next_agent_ping_ms_{0};
};

const char *ToString(MicroRosConnectionState state);

} // namespace fault_detector_sensor
