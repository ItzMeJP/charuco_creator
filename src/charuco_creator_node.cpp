/**\file charuco_detector_node.cpp
 * \brief Main for the ChArUco board creating
 *
 * @version 1.0
 * @author João Pedro Carvalho de Souza
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <ros/ros.h>
#include <charuco_creator/charuco_creator.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ###################################################################################   <main>   ##############################################################################
int main(int argc, char **argv) {
	ros::init(argc, argv, "charuco_creator");

	ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

	charuco_creator::ChArUcoCreator chArUcoCreator;
	chArUcoCreator.boardDataFromParameterServer(private_node_handle);
	chArUcoCreator.createBoard();

	//ros::spin();

	return 0;
}
// ###################################################################################   </main>   #############################################################################
