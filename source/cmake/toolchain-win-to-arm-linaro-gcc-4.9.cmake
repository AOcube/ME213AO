# use with -DCMAKE_TOOLCHAIN_FILE="E:/cosmos-source/core/cmake/toolchain-win-to-arm-gcc-4.9"
# name of the target system
SET(CMAKE_SYSTEM_NAME Linux)

SET(CMAKE_C_COMPILER     "C:/Program Files (x86)/Linaro/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09/bin/arm-linux-gnueabihf-gcc.exe")
SET(CMAKE_CXX_COMPILER   "C:/Program Files (x86)/Linaro/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09/bin/arm-linux-gnueabihf-g++.exe")
SET(CMAKE_FIND_ROOT_PATH "C:/Program Files (x86)/Linaro/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09/arm-linux-gnueabihf")

SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
