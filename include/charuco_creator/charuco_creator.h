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
#include <string>

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

		double	create_squares_size_,
				create_markers_size_;

		bool create_showImage_;

		std::string create_output_path_;

		int  	create_x_squares_,
		       	create_y_squares_,
				create_px_img_size_x_,
				create_px_img_size_y_,
				create_margins_,
				create_borderBits_,
				create_dictionaryId_;

	};

}
