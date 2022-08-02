#source devel/setup.zsh

sudo modprobe gs_usb;

sudo ip link set can0 up type can bitrate 500000;

roslaunch tracer_bringup tracer_robot_base.launch;
`
