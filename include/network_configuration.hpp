#pragma once

#include <Arduino.h>

#include <cstdint>

namespace fault_detector_sensor {

struct NetworkConfiguration {
  String wifi_ssid;
  String wifi_password;
  String agent_address{"192.168.178.69"};
  std::uint16_t agent_port{8888};

  bool has_wifi_configuration() const;
  bool has_agent_configuration() const;
};

class NetworkConfigurationStore {
public:
  bool Load(NetworkConfiguration &configuration) const;
  bool SaveWifi(const String &ssid, const String &password,
                NetworkConfiguration &configuration) const;
  bool SaveAgent(const String &address, std::uint16_t port,
                 NetworkConfiguration &configuration) const;
  bool Reset(NetworkConfiguration &configuration) const;
};

bool IsValidWifiConfiguration(const String &ssid, const String &password);
bool IsValidAgentConfiguration(const String &address, std::uint16_t port);

} // namespace fault_detector_sensor
