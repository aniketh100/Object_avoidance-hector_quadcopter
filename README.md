--Using HECTOR QUADROTOR for ROS Noetic, we have added LIDAR in the URDF File and enabled both camera and LIDAR to work together to determine distances from objects.
    - Uses OpenCV for image processing
    - Distance can be set separately
    - Converts camera feed to gaussian blur and grayscale
    - Determines distance based on pixel intensity.
    - WIP
