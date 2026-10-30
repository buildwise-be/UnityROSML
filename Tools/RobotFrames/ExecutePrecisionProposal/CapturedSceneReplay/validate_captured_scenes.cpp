#include "ur16e_mtc_gpio_stage/plan_validation.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>

moveit_msgs::msg::PlanningScene loadScene(const char *path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("Could not open scene CDR file");
  const std::vector<char> bytes((std::istreambuf_iterator<char>(input)), {});
  if (bytes.empty()) throw std::runtime_error("Scene CDR file is empty");
  rclcpp::SerializedMessage serialized(bytes.size());
  auto &data = serialized.get_rcl_serialized_message();
  std::memcpy(data.buffer, bytes.data(), bytes.size());
  data.buffer_length = bytes.size();
  rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serializer;
  moveit_msgs::msg::PlanningScene scene;
  serializer.deserialize_message(&serialized, &scene);
  return scene;
}

int main(int argc, char **argv)
{
  if (argc != 3) return 2;
  const auto expected = loadScene(argv[1]);
  const auto current = loadScene(argv[2]);
  const auto reason = ur16e_mtc_gpio_stage::validatePlanStart(expected, current, 0.01);
  std::cout << "Captured scene validator result: " << (reason.empty() ? "PASS" : reason) << '\n';
  std::cout << "Planned/current world objects: " << expected.world.collision_objects.size()
            << '/' << current.world.collision_objects.size() << '\n';
  return reason.empty() ? 0 : 1;
}
