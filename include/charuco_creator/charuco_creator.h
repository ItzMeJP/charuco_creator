#pragma once

/**\file charuco_creator.h
 * \brief Creator of ChArUco patterns
 *
 * @version 1.0
 * @author João Pedro Carvalho de Souza
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// external libs includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// ros includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace charuco_creator{

	class ChArUcoCreator {
	public:
		ChArUcoCreator() {}
		~ChArUcoCreator() {}

		virtual void boardDataFromParameterServer(ros::NodeHandlePtr &_private_node_handle);
		virtual void createBoard();

	protected:

		ros::NodeHandlePtr node_handle_;
		ros::NodeHandlePtr private_node_handle_;

		double	create_squares_size,
				create_markers_size;

		bool create_showImage;

		std::string create_output_path;

		int  	create_x_squares,
		       	create_y_squares,
				create_px_img_size_x,
				create_px_img_size_y,
				create_margins,
				create_borderBits,
				create_dictionaryId;

	};

}
