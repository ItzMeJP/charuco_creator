/**\file charuco_detector.cpp
 * \brief Detector of ChArUco patterns
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <charuco_creator/charuco_creator.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace charuco_creator {

	void ChArUcoCreator::boardDataFromParameterServer(ros::NodeHandlePtr &_private_node_handle){
		private_node_handle_ = _private_node_handle;
		private_node_handle_->param("charuco_create_board/x_squares", create_x_squares, 10);
		private_node_handle_->param("charuco_create_board/y_squares", create_y_squares, 14);
		private_node_handle_->param("charuco_create_board/squares_size", create_squares_size, 0.0280);
		private_node_handle_->param("charuco_create_board/markers_size", create_markers_size, 0.0168);
		private_node_handle_->param("charuco_create_board/px_img_size_x", create_px_img_size_x, 600);
		private_node_handle_->param("charuco_create_board/px_img_size_y", create_px_img_size_y, 500);
		private_node_handle_->param("charuco_create_board/output_path", create_output_path, std::string(""));
		private_node_handle_->param("charuco_create_board/margins", create_margins, 10);
		private_node_handle_->param("charuco_create_board/borderBits", create_borderBits, 1);
		private_node_handle_->param("charuco_create_board/showImage", create_showImage, true);
		private_node_handle_->param("charuco_create_board/dictionaryId", create_dictionaryId, 10);

	}

 	void ChArUcoCreator::createBoard(){

		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(create_dictionaryId));
		cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(create_x_squares, create_y_squares, (float)create_squares_size,(float)create_markers_size, dictionary);
		cv::Mat boardImage;

		board->draw(cv::Size(create_px_img_size_x, create_px_img_size_y), boardImage, create_margins, create_borderBits);
		if(create_showImage) {
			imshow("board", boardImage);
			cv::waitKey(0);
		}
		if(create_output_path.empty()){
			ROS_INFO("Board Output Path: Current Folder");
			cv::imwrite("created_board.jpg", boardImage);
		}
		else{
			ROS_INFO("Board Output Path: %s", create_output_path.c_str());
			cv::imwrite((create_output_path + "/created_board.jpg").c_str(), boardImage);
		}
	}



}