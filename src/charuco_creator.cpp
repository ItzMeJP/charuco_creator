/**\file charuco_detector.cpp
 * \brief Creator of ChArUco patterns
 *
 * @version 1.0
 * @author João Pedro Carvalho de Souza
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <charuco_creator/charuco_creator.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace charuco_creator {

	void ChArUcoCreator::boardDataFromParameterServer(ros::NodeHandlePtr &_private_node_handle){
		private_node_handle_ = _private_node_handle;
		private_node_handle_->param("charuco_create_board/x_squares", create_x_squares_, 10);
		private_node_handle_->param("charuco_create_board/y_squares", create_y_squares_, 14);
		private_node_handle_->param("charuco_create_board/squares_size", create_squares_size_, 0.0280);
		private_node_handle_->param("charuco_create_board/markers_size", create_markers_size_, 0.0168);
		private_node_handle_->param("charuco_create_board/px_img_size_x", create_px_img_size_x_, 600);
		private_node_handle_->param("charuco_create_board/px_img_size_y", create_px_img_size_y_, 500);
		private_node_handle_->param("charuco_create_board/output_path", create_output_path_, std::string(""));
		private_node_handle_->param("charuco_create_board/margins", create_margins_, 10);
		private_node_handle_->param("charuco_create_board/borderBits", create_borderBits_, 1);
		private_node_handle_->param("charuco_create_board/showImage", create_showImage_, true);
		private_node_handle_->param("charuco_create_board/dictionaryId", create_dictionaryId_, 10);

	}

 	void ChArUcoCreator::createBoard(){

		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(create_dictionaryId_));
		cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(create_x_squares_, create_y_squares_, (float)create_squares_size_,(float)create_markers_size_, dictionary);
		cv::Mat boardImage;

		board->draw(cv::Size(create_px_img_size_x_, create_px_img_size_y_), boardImage, create_margins_, create_borderBits_);
		if(create_showImage_) {
			imshow("board", boardImage);
			cv::waitKey(0);
		}
		if(create_output_path_.empty()){
			const char* path  = std::getenv("HOME");
			cv::imwrite((std::string)path + "/created_board.jpg" , boardImage);
			ROS_INFO("Board Output Path: $HOME");
		}
		else{
			ROS_INFO("Board Output Path: %s", create_output_path_.c_str());
			cv::imwrite((create_output_path_ + "created_board.jpg").c_str(), boardImage);
		}
	}



}
