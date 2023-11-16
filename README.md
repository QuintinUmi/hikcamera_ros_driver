# hikcamera_ros_driver
A driver for hikvision camera in ros

```
hikcamera_opr
    │  CMakeLists.txt
    │  package.xml
    │
    ├─config
    │      hikcamera.yaml
    │      ros-node-config.yaml
    │
    ├─include
    │      hikcamera.h
    │      hikcameraDataType.h
    │      kbhit.h
    │
    ├─launch
    │      img_hikcamera_grab_save.launch
    │      msg_hikcamera_grab.launch
    │
    ├─lib
    │      hikcamera.cpp
    │
    └─src
            img_hikcamera_grab_save.cpp
            img_show.cpp
            key_input.cpp
            msg_hikcamera_grabbing.cpp
```
