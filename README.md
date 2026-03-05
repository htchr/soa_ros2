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

- [ ] bi moveit config
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

# working rosetta lerobot inference with the following packages

- lerobot: 0f551df8f4bad4c504e395ea3df74fc5f714016f
- feetech_ros2_driver: e424839f0fb23cc0b0ade8d6e5ea166e30811356
- rosetta: e0e6be11a80f6a75f055d6e0ed20fef39434485e
- lerobot-robot-rosetta: e4ed91ee5cd58c0163dabf0af699d3e3c1f193dc
- lerobot-teleoperator-rosetta cc2f76b326f6c8bb12ee519a0980c19286c994e9
- rosetta_interfaces: 90e979108b9b8cdbe791697ee9a338192e16e9fc


