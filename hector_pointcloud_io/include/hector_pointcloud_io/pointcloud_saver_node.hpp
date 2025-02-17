#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace hector_pointcloud_io
{

template<typename C>
struct is_vector : std::false_type {
};

template<typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {
};

template<typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

class PointcloudSaver : public rclcpp::Node
{

public:
  explicit PointcloudSaver( const rclcpp::NodeOptions &options );

private:
  /**
   * @brief Declares and loads a ROS parameter
   *
   * @param name name
   * @param param parameter variable to load into
   * @param description description
   * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
   * @param is_required whether failure to load parameter will stop node
   * @param read_only set parameter to read-only
   * @param from_value parameter range minimum
   * @param to_value parameter range maximum
   * @param step_value parameter range step
   * @param additional_constraints additional constraints description
   */
  template<typename T>
  void declareAndLoadParameter( const std::string &name, T &param, const std::string &description,
                                bool add_to_auto_reconfigurable_params = true,
                                bool is_required = false, bool read_only = false,
                                const std::optional<double> &from_value = std::nullopt,
                                const std::optional<double> &to_value = std::nullopt,
                                const std::optional<double> &step_value = std::nullopt,
                                const std::string &additional_constraints = "" );

  rcl_interfaces::msg::SetParametersResult
  parametersCallback( const std::vector<rclcpp::Parameter> &parameters );

  void setup();

  void serviceCallback( const std_srvs::srv::Trigger::Request::SharedPtr &request,
                        const std_srvs::srv::Trigger::Response::SharedPtr &response );

  std::vector<std::tuple<std::string, std::function<void( const rclcpp::Parameter & )>>>
      auto_reconfigurable_params_;

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_server_;

  std::string topic_ = "pointcloud";
  std::string output_folder_;
  std::string output_format_ = "pcd";
  std::string output_filename_prefix_ = "pointcloud";
  int timeout_ms_ = 1000;
};

} // namespace hector_pointcloud_io
