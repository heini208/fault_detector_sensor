#include "network_configuration.hpp"

#include <IPAddress.h>
#include <Preferences.h>

namespace fault_detector_sensor {
namespace {

constexpr char kPreferencesNamespace[] = "fault_sensor";
constexpr char kWifiSsidKey[] = "wifi_ssid";
constexpr char kWifiPasswordKey[] = "wifi_pass";
constexpr char kAgentAddressKey[] = "agent_addr";
constexpr char kAgentPortKey[] = "agent_port";

bool OpenPreferences(Preferences &preferences, bool read_only) {
  return preferences.begin(kPreferencesNamespace, read_only);
}

} // namespace

bool NetworkConfiguration::has_wifi_configuration() const {
  return IsValidWifiConfiguration(wifi_ssid, wifi_password);
}

bool NetworkConfiguration::has_agent_configuration() const {
  return IsValidAgentConfiguration(agent_address, agent_port);
}

bool NetworkConfigurationStore::Load(
    NetworkConfiguration &configuration) const {
  Preferences preferences;
  if (!OpenPreferences(preferences, false)) {
    return false;
  }

  if (preferences.isKey(kWifiSsidKey)) {
    configuration.wifi_ssid = preferences.getString(kWifiSsidKey);
  }
  if (preferences.isKey(kWifiPasswordKey)) {
    configuration.wifi_password = preferences.getString(kWifiPasswordKey);
  }
  if (preferences.isKey(kAgentAddressKey)) {
    configuration.agent_address = preferences.getString(kAgentAddressKey);
  }
  if (preferences.isKey(kAgentPortKey)) {
    configuration.agent_port = preferences.getUShort(kAgentPortKey);
  }
  preferences.end();

  if (!configuration.has_agent_configuration()) {
    configuration.agent_address = "192.168.178.69";
    configuration.agent_port = 8888;
  }
  return true;
}

bool NetworkConfigurationStore::SaveWifi(
    const String &ssid, const String &password,
    NetworkConfiguration &configuration) const {
  if (!IsValidWifiConfiguration(ssid, password)) {
    return false;
  }

  Preferences preferences;
  if (!OpenPreferences(preferences, false)) {
    return false;
  }
  const bool saved_ssid = preferences.putString(kWifiSsidKey, ssid) > 0;
  const bool saved_password =
      preferences.putString(kWifiPasswordKey, password) > 0;
  preferences.end();

  if (!saved_ssid || !saved_password) {
    return false;
  }
  configuration.wifi_ssid = ssid;
  configuration.wifi_password = password;
  return true;
}

bool NetworkConfigurationStore::SaveAgent(
    const String &address, std::uint16_t port,
    NetworkConfiguration &configuration) const {
  if (!IsValidAgentConfiguration(address, port)) {
    return false;
  }

  Preferences preferences;
  if (!OpenPreferences(preferences, false)) {
    return false;
  }
  const bool saved_address =
      preferences.putString(kAgentAddressKey, address) > 0;
  const bool saved_port = preferences.putUShort(kAgentPortKey, port) > 0;
  preferences.end();

  if (!saved_address || !saved_port) {
    return false;
  }
  configuration.agent_address = address;
  configuration.agent_port = port;
  return true;
}

bool NetworkConfigurationStore::Reset(
    NetworkConfiguration &configuration) const {
  Preferences preferences;
  if (!OpenPreferences(preferences, false)) {
    return false;
  }
  const bool cleared = preferences.clear();
  preferences.end();

  if (cleared) {
    configuration = NetworkConfiguration{};
  }
  return cleared;
}

bool IsValidWifiConfiguration(const String &ssid, const String &password) {
  return !ssid.isEmpty() && ssid.length() <= 32 && password.length() <= 64;
}

bool IsValidAgentConfiguration(const String &address, std::uint16_t port) {
  IPAddress parsed_address;
  return port != 0 && parsed_address.fromString(address);
}

} // namespace fault_detector_sensor
