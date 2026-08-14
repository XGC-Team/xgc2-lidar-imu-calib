cmake_minimum_required(VERSION 3.0.2)
project(lidar_imu_calibration)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/identify_lidar_imu_extrinsic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS scripts/record_calib_bag.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
