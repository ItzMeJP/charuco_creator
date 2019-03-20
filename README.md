# charuco_creator
ROS Package to create [ChAruco patterns](https://docs.opencv.org/master/df/d4a/tutorial_charuco_detection.html).

The charuco_detector package can be found at: [charuco_detector](https://github.com/carlosmccosta/charuco_detector.git)

## How to use
Set the config.yaml with the parameters of the new ChAruco Pattern and run the launch:

$roslaunch charuco_creator generate_charuco.launch

The new charuco pattern will be generated in the specific path indicate in the config.yaml, otherwise in $HOME folder.