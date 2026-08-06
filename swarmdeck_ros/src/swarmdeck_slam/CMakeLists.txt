cmake_minimum_required(VERSION 3.8)
project(swarmdeck_slam)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# The source file must be committed with its executable bit set. `install(PROGRAMS)`
# would normally set it on the copy, but this workspace builds `--symlink-install`,
# so the install tree points at the source and inherits ITS mode. A non-executable
# source yields "executable 'covariance_relay.py' not found on the libexec
# directory" — a message that points at the install tree, where the symlink is in
# fact present and correct.
install(PROGRAMS nodes/covariance_relay.py DESTINATION lib/${PROJECT_NAME})
ament_package()
