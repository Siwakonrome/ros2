cmake_minimum_required(VERSION 3.5)
project(turtlesim_plus)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
######################################################



############################################################################################################
# Find Python executable
find_program(PYTHON_EXECUTABLE python3)

if(NOT PYTHON_EXECUTABLE)
  message(FATAL_ERROR "Python 3 not found. Please install Python 3 to proceed.")
endif()

# Find pip, or install it if not found
find_program(PIP_EXECUTABLE pip3)

if(NOT PIP_EXECUTABLE)
  message(STATUS "pip not found. Attempting to install pip...")

  # Try to install pip using get-pip.py if ensurepip is not available
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import urllib.request; urllib.request.urlretrieve('https://bootstrap.pypa.io/get-pip.py', 'get-pip.py')"
    RESULT_VARIABLE DOWNLOAD_GET_PIP_RESULT
  )

  if(NOT DOWNLOAD_GET_PIP_RESULT EQUAL "0")
    message(FATAL_ERROR "Failed to download get-pip.py.")
  endif()

  # Install pip locally
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} get-pip.py --user
    RESULT_VARIABLE RUN_GET_PIP_RESULT
  )

  if(NOT RUN_GET_PIP_RESULT EQUAL "0")
    message(FATAL_ERROR "Failed to install pip using get-pip.py.")
  else()
    # Add ~/.local/bin to PATH if pip is installed locally
    set(LOCAL_PIP_PATH "$ENV{HOME}/.local/bin")
    if(EXISTS ${LOCAL_PIP_PATH})
      message(STATUS "Adding ${LOCAL_PIP_PATH} to PATH.")
      set(ENV{PATH} "$ENV{PATH}:${LOCAL_PIP_PATH}")
      find_program(PIP_EXECUTABLE pip3 PATHS ${LOCAL_PIP_PATH})
    endif()
  endif()

  # Clean up get-pip.py file
  file(REMOVE get-pip.py)
endif()

if(NOT PIP_EXECUTABLE)
  message(FATAL_ERROR "Failed to install pip.")
endif()

# Install Python dependencies from requirements.txt
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt")
  execute_process(
    COMMAND ${PIP_EXECUTABLE} install --user -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
    RESULT_VARIABLE PIP_INSTALL_RESULT
  )
  if(NOT PIP_INSTALL_RESULT EQUAL "0")
    message(FATAL_ERROR "Failed to install Python dependencies using pip.")
  endif()
endif()
############################################################################################################


# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/turtlesim_plus_node.py
  scripts/pizza_on_click.py
  DESTINATION lib/${PROJECT_NAME}
)


################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  DESTINATION share/${PROJECT_NAME})

ament_package()
