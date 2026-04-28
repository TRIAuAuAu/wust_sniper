#pragma once
#include <mqtt/async_client.h>
#include <string>

namespace doorlock_sniper {

class MqttClient {
public:
  MqttClient(const std::string &id, const std::string &ip, int port);

  void publish(const std::string &topic, const std::string &payload);

private:
  std::unique_ptr<mqtt::async_client> client_;
  mqtt::connect_options conn_opts_;
};

} // namespace doorlock_sniper