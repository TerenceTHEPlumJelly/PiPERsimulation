# PiPERsimulation
基于松灵PiPER机械臂的双臂数据进行的MATLAB仿真程序
一个基于松灵PiPER机械臂的RRRRRR Manipulator的模拟，使得模拟机械臂可以利用输入的结构体数组来进行相应的动作
动作使用figure窗口进行显示，机械臂底座被圆柱体表示
机械臂包括6个机械臂角度和最后一个 gripper 作为抓夹的开合量
支持的输入格式为ROS/机器人控制系统里标准的 JointState 消息格式，应包含：
  names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
  pos: 每个关节的位置（角度 or 位移）
  vel: 速度（可选）
  effort: 力矩/力（可选）
程序为完整体现机械臂的动作进行了视野的修改  
左右臂基座初始坐标与方向：左右臂基座中心连线的中点为坐标系原点，左臂在y轴负半轴，右臂在正半轴，左右臂均以朝前为x轴正方向

基于ChatGPT生成

## actionBuild模块
用于解析单个动作

## actionRead模块
可以读取PiPER双臂日志
通过将读取到的动作流调用actionBuild转变为连续动作

