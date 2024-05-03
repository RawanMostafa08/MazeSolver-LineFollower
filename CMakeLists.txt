cmake_minimum_required(VERSION 3.0.0)

project(image_processing VERSION 0.1.0 LANGUAGES C CXX)
set(OpenCV_DIR "D:/opencv/build")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(include)

set(SOURCES 
    main.cpp
    car.cpp
    utils.cpp
    lines.cpp
)

add_executable(image_processing ${SOURCES})  
target_link_libraries(image_processing ${OpenCV_LIBS} )
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
