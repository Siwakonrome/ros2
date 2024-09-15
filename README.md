# Creating custom msg and srv files [tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

[Video Lectures](https://drive.google.com/drive/folders/1jGeKOIWiad_GzL5rlh3OnR4Bi1f7OaLy?usp=sharing).

## Create a new package
   ```bash
   ros2 pkg create --build-type ament_cmake --license Apache-2.0 tutorial_interfaces
   ```
   ```bash
   mkdir msg srv
   ```
## Create custom definitions
- msg definition
   ```bash
   geometry_msgs/Point center
   float64 radius
   ```
- srv definition
   ```bash
   int64 a
   int64 b
   int64 c
   ---
   int64 sum
   ```
## CMakeLists.txt
   ```bash
   find_package(geometry_msgs REQUIRED)
   find_package(rosidl_default_generators REQUIRED)
    
   rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/Num.msg"
      "msg/Sphere.msg"
      "srv/AddThreeInts.srv"
      DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
   )
   ```
## package.xml
   ```bash
   <depend>geometry_msgs</depend>
   <buildtool_depend>rosidl_default_generators</buildtool_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```
