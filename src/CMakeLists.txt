# 增加一个可执行的二进制
ADD_EXECUTABLE( main main.cpp )

# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

# 增加opencv的依赖
FIND_PACKAGE( OpenCV 2.4 REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# INCLUDE_DIRECTORIES( ${PROJECT_SOURSE_DIR}/include )

ADD_LIBRARY( slambase slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )

ADD_EXECUTABLE( DetectFeatures DetectFeatures.cpp )
TARGET_LINK_LIBRARIES( DetectFeatures 
    slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )
