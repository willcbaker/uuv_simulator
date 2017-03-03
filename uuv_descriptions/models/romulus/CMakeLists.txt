cmake_minimum_required(VERSION 2.8.3)
find_package(catkin REQUIRED)

set(PACKAGE_NAME romulus)

install(DIRECTORY mesh launch robots urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PACKAGE_NAME}
        PATTERN "*~" EXCLUDE)
