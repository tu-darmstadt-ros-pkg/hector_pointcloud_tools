//
// Created by stefan on 12.02.25.
//

#ifndef POINTCLOUD_ACCUMULATOR_LOGGING_HPP
#define POINTCLOUD_ACCUMULATOR_LOGGING_HPP

#include <rclcpp/logging.hpp>

#define PCA_LOGGER rclcpp::get_logger( "hector_pointcloud_accumulator" )
#define PCA_LOG_DEBUG( ... ) RCLCPP_DEBUG( PCA_LOGGER, __VA_ARGS__ )
#define PCA_LOG_INFO( ... ) RCLCPP_INFO( PCA_LOGGER, __VA_ARGS__ )
#define PCA_LOG_WARN( ... ) RCLCPP_WARN( PCA_LOGGER, __VA_ARGS__ )
#define PCA_LOG_ERROR( ... ) RCLCPP_ERROR( PCA_LOGGER, __VA_ARGS__ )

#define PCA_LOG_INFO_ONCE( ... ) RCLCPP_INFO_ONCE( PCA_LOGGER, __VA_ARGS__ )
#define PCA_LOG_WARN_ONCE( ... ) RCLCPP_WARN_ONCE( PCA_LOGGER, __VA_ARGS__ )
#define PCA_LOG_ERROR_ONCE( ... ) RCLCPP_ERROR_ONCE( PCA_LOGGER, __VA_ARGS__ )

#endif // POINTCLOUD_ACCUMULATOR_LOGGING_HPP
