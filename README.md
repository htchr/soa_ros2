# Standard Open Arm ROS2

## Install

**Direct**

```bash
cd ros2_ws/src
git clone --recursive <>
./soa_ros2/build.sh
```

**Docker**

1. Install Docker
1. Install the NVIDIA container toolkit
1. Install the Remote Development VS Code extension
1. Open the repo in VS Code
1. Select "Reopen in Container"


## Usage

1. Calibrate the arm using lerobot
1. Update the config settings in the `soa_bringup` package
1. Bringup using one of the following launch files

**Single Arm Bringup**

```bash
ros2 launch soa_bringup soa_bringup.launch.py
```
Arguments:  
`leader:=true` bringup the leader arm and run the teleop node  
`display:=true` run Rviz2 with the soa config  
`controller_mode:=forward` launch with either the forward command controller or joint trajectory controller

**Single Arm Moveit**

```bash
ros2 launch soa_moveit_config soa_moveit_bringup.launch.py
```
Launch the follower arm using the joint trajectory controller and Moveit

**Bi-manual Arm Bringup**

```bash
ros2 launch soa_bringup bi_soa_bringup.launch.py
```
Arguments:  
`leader:=true` bringup the leader arms and run the teleop node  
`display:=true` run Rviz2 with the soa config  
`controller_mode:=forward` launch with either the forward command controller or joint trajectory controller

**Bi-manual Arm Moveit**

```bash
ros2 launch bi_soa_moveit_config bi_soa_moveit_bringup.launch.py
```
Launch dual follower arms using the joint trajectory controller and Moveit


### Rosetta

The ros2 rosetta packages are used to run VLA models alongside ros for rapid controller switching between learned and classical methods.  
Below are example commands to use a VLA model trained using lerobot with the SOA in ros2:

Terminal 1:
```bash
ros2 launch soa_bringup soa_bringup.launch.py
```

Terminal 2:
```bash
ros2 launch rosetta rosetta_client_launch.py \
contract_path:=./soa_rosetta/contracts/soa_act_contract.yaml \
pretrained_name_or_path:=</path/to/your/pretrained_model>
```

Terminal 3:
```bash
ros2 action send_goal /run_policy \
rosetta_interfaces/action/RunPolicy "{prompt: 'your prompt'}"
```


## TODO

- [ ] bi soa rosetta
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


