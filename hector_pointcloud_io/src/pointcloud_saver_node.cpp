#include "hector_pointcloud_io/pointcloud_saver_node.hpp"
#include "hector_pointcloud_io/pointcloud_io.hpp"

#include <functional>
#include <rclcpp/wait_for_message.hpp>

namespace hector_pointcloud_io
{

PointcloudSaver::PointcloudSaver( const rclcpp::NodeOptions &options )
    : Node( "pointcloud_saver", options )
{
  this->declareAndLoadParameter( "topic", topic_, "Topic to listen for pointclouds on", true, false,
                                 false );
  this->declareAndLoadParameter( "output_folder", output_folder_, "Folder to save pointclouds to",
                                 true, true, false );
  this->declareAndLoadParameter( "output_format", output_format_,
                                 "Output format of pointclouds. Supported: pcd, ifs, ply, vtk",
                                 true, false, false );
  this->declareAndLoadParameter(
      "output_filename_prefix", output_filename_prefix_,
      "Prefix for output filenames. Name will be <prefix>.<timestamp>.<output_format>", true, false,
      false );
  this->declareAndLoadParameter( "timeout_ms", timeout_ms_,
                                 "Timeout for waiting for pointclouds in ms.", true, false, false,
                                 0, 30000, 50 );

  this->setup();
}

template<typename T>
void PointcloudSaver::declareAndLoadParameter(
    const std::string &name, T &param, const std::string &description,
    const bool add_to_auto_reconfigurable_params, const bool is_required, const bool read_only,
    const std::optional<double> &from_value, const std::optional<double> &to_value,
    const std::optional<double> &step_value, const std::string &additional_constraints )
{

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue( param ).get_type();

  if ( from_value.has_value() && to_value.has_value() ) {
    if constexpr ( std::is_integral_v<T> ) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>( step_value.has_value() ? step_value.value() : 1 );
      range.set__from_value( static_cast<T>( from_value.value() ) )
          .set__to_value( static_cast<T>( to_value.value() ) )
          .set__step( step );
      param_desc.integer_range = { range };
    } else if constexpr ( std::is_floating_point_v<T> ) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = static_cast<T>( step_value.has_value() ? step_value.value() : 1.0 );
      range.set__from_value( static_cast<T>( from_value.value() ) )
          .set__to_value( static_cast<T>( to_value.value() ) )
          .set__step( step );
      param_desc.floating_point_range = { range };
    } else {
      RCLCPP_WARN( this->get_logger(),
                   "Parameter type of parameter '%s' does not support specifying a range",
                   name.c_str() );
    }
  }

  this->declare_parameter( name, type, param_desc );

  try {
    param = this->get_parameter( name ).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr ( is_vector_v<T> ) {
      ss << "[";
      for ( const auto &element : param )
        ss << element << ( &element != &param.back() ? ", " : "]" );
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM( this->get_logger(), ss.str() );
  } catch ( rclcpp::exceptions::ParameterUninitializedException & ) {
    if ( is_required ) {
      RCLCPP_FATAL_STREAM( this->get_logger(),
                           "Missing required parameter '" << name << "', exiting" );
      exit( EXIT_FAILURE );
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr ( is_vector_v<T> ) {
        ss << "[";
        for ( const auto &element : param )
          ss << element << ( &element != &param.back() ? ", " : "]" );
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM( this->get_logger(), ss.str() );
      this->set_parameters( { rclcpp::Parameter( name, rclcpp::ParameterValue( param ) ) } );
    }
  }

  if ( add_to_auto_reconfigurable_params ) {
    std::function<void( const rclcpp::Parameter & )> setter =
        [&param]( const rclcpp::Parameter &p ) { param = p.get_value<T>(); };
    auto_reconfigurable_params_.push_back( std::make_tuple( name, setter ) );
  }
}

/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
rcl_interfaces::msg::SetParametersResult
PointcloudSaver::parametersCallback( const std::vector<rclcpp::Parameter> &parameters )
{

  for ( const auto &param : parameters ) {
    for ( auto &[name, callback] : auto_reconfigurable_params_ ) {
      if ( name == "output_format" ) {
        if ( std::set<std::string>{ "pcd", "ifs", "ply", "vtk" }.count( param.as_string() ) == 0 ) {
          RCLCPP_ERROR( this->get_logger(), "Invalid output format '%s'", param.as_string().c_str() );
          rcl_interfaces::msg::SetParametersResult result;
          result.reason = "Invalid output format";
          result.successful = false;
          return result;
        }
      }
      if ( param.get_name() == name ) {
        callback( param );
        RCLCPP_INFO( this->get_logger(), "Reconfigured parameter '%s'", param.get_name().c_str() );
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

/**
 * @brief Sets up subscribers, publishers, etc. to configure the node
 */
void PointcloudSaver::setup()
{

  // callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind( &PointcloudSaver::parametersCallback, this, std::placeholders::_1 ) );

  // service server for handling service calls
  service_server_ = this->create_service<std_srvs::srv::Trigger>(
      "~/save_pointcloud", std::bind( &PointcloudSaver::serviceCallback, this,
                                      std::placeholders::_1, std::placeholders::_2 ) );
}

void PointcloudSaver::serviceCallback( const std_srvs::srv::Trigger::Request::SharedPtr &,
                                       const std_srvs::srv::Trigger::Response::SharedPtr &response )
{
  RCLCPP_INFO( this->get_logger(), "Received request to save pointcloud." );
  sensor_msgs::msg::PointCloud2 pointcloud;
  if ( !rclcpp::wait_for_message( pointcloud, this->shared_from_this(), topic_,
                                  std::chrono::milliseconds( timeout_ms_ ) ) ) {
    RCLCPP_ERROR( this->get_logger(), "Timeout while waiting for pointcloud message." );
    response->success = false;
    return;
  }
  std::string timestamp =
      std::to_string( rclcpp::Time( pointcloud.header.stamp ).nanoseconds() / 1000 );
  std::string path =
      output_folder_ + "/" + output_filename_prefix_ + "." + timestamp + "." + output_format_;
  if ( !save_pointcloud( path, pointcloud ) ) {
    RCLCPP_ERROR( this->get_logger(), "Failed to write pointcloud to file." );
    response->success = false;
    return;
  }
  RCLCPP_INFO( this->get_logger(), "Saved pointcloud." );
  response->success = true;
}
} // namespace hector_pointcloud_io

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE( hector_pointcloud_io::PointcloudSaver )
