# Standard Open Arm ROS2

## Install

### Direct

```bash
cd ros2_ws/src
git clone --recursive <>
rosdep install --from-paths . --ignore-src -y
```

### Docker

## Usage

1. Update the config settings in the `soa_bringup` package


## TODO

- [ ] soa moveit config
- [ ] bi moveit config
- [ ] soa bringup rviz config w/ cameras
- [ ] bi bringup rviz config w/ cameras
- [ ] soa teleop node
- [ ] bi teleop node
- [ ] soa rosetta node(s)
- [ ] bi soa rosetta node(s)
- [ ] soa gazebo sim
- [ ] bi gazebo sim


## Acknowledgements 

- [The Robot Studio SO101](https://github.com/TheRobotStudio/SO-ARM100)
- [HuggingFace Lerobot SO101](https://huggingface.co/docs/lerobot/en/so101)
- [JafarAbdi feetech_ros2_driver](https://github.com/JafarAbdi/feetech_ros2_driver)
- [JafarAbdi ros2_so_arm](https://github.com/JafarAbdi/ros2_so_arm)
- [iblnkn rosetta](https://github.com/iblnkn/rosetta)
- [nimiCurtis so101_ros2](https://github.com/nimiCurtis/so101_ros2)


## License

[LICENSE](LICENSE)
