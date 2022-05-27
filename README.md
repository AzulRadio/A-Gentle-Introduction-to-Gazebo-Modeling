English Translation is on the way.

[中文版点此](./Tutorial_CN.md)

90%的Gazebo模型教程从类似这样的命令行指令开始：

`gazebo worlds/pioneer2dx.world` ([Gazebo 官方教程](https://classic.gazebosim.org/tutorials))

或是类似这样的：

`roslaunch my_simulations my_world.launch` ([Construct 的 《五分钟Gazebo》 系列教程](https://www.theconstructsim.com/category/gazebo-tutorials/))

上面这些命令是将一些现成的模型文件刷进Gazebo，作为新手的导入并无不妥，但是所有这些教程都没有回答一个至关重要的问题：**模型要从哪里来**。

你确实可以从许多网站上找到出自专业人士之手的3D模型，如果你运气好，你甚至可以找到你需要的 URDF（模型）文件，但是你需要的只是一个简单的，用于演示的样例模型呢？这些教程无一例外地忽视了这个问题，这也是我写这个教程的初衷，**如何搭建/修改一个你需要的 Gazebo 场景**。

本文的目标读者如下：
- 知道 ROS topic 是什么，会用 `rosrun`, `roslaunch`，能依葫芦画瓢写简单的 `roslaunch` 文件
- 不以建模为最终目的，即，模型是你用于一个更大目标的一部分（这不是一个建模教程！）
- 死线逐渐逼近，你需要一个仿真演示来交差，但是你找不到现成的 URDF
- 你已经有了一个别人做的 URDF/XACRO ，但是不知道怎么修改它

本文所涵盖的内容如下：
- Gazebo 需要什么格式的模型
- 如何将 URDF 导入 Gazebo
- 如何制作简单的 URDF
- 如何将 URDF 设为不动的，透明的，没有碰撞体积的，不受重力影响的
- 如何使用 CAD 软件生成 URDF (Solidworks 或 Blender)
- 如何制作更精细的 URDF (mesh 网格文件)
- 如何制作有贴图的 URDF (texture 纹理 和 material 材质)
- 如何改变 URDF 在 Gazebo 中的速度和位置 (Gazebo Plugin)
- 如何在 Gazebo 中转动 URDF 的关节 (ROS Moveit!)
- 如何制作仿真相机或仿真传感器 (Gazebo Plugin)

未来可能的更多内容：
- 如何抓举：以 Universal Robot 为例
- 如何在 Gazebo 中使用人物模型
