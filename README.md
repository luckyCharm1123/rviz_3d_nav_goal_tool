🤔 为什么需要这个插件？
使用过 RViz2 的同学都知道，原生工具栏里的 2D Goal Pose 虽然好用，但它有一个硬伤：只能在 XY 平面上设定目标，无法调节 Z 轴（高度）。

如果你是在做：

🚁 空中机器人（无人机）：需要指定目标飞行高度。

🦾 机械臂：末端执行器需要到达三维空间中的特定点。

🌊 水下机器人：需要指定深度。

原生的 2D 工具就显得捉襟见肘了。你通常需要手动发布话题消息，或者写一个专门的脚本来发点，效率极低。

为了解决这个问题，我们将经典的 rviz-3d-nav-goal-tool 原生移植到了 ROS 2 (Humble/Iron/Rolling) 环境中！
✨ 插件核心功能
这款插件是一个标准的 RViz2 Panel 工具，加载后可以在工具栏直接使用。

1. 交互式 3D 目标设置
它延续了最直观的鼠标交互逻辑，无需键盘输入坐标：

第 1 步（定平面位置与朝向）：鼠标左键点击并拖动，此时表现与普通的 2D Goal 一样，确定 X、Y 坐标和 Yaw 偏航角。

第 2 步（定高度，这是重点！）：在左键拖动不松手的情况下，点击鼠标右键并上下拖动，即可直观地调整 Z 轴高度。

第 3 步（发布）：松开鼠标，工具会自动向指定话题发布标准的 geometry_msgs/msg/PoseStamped 消息。

2. 可视化反馈
在操作过程中，插件会在 RViz2 场景中生成一系列绿色箭头，实时显示当前选定的高度层级，所见即所得。

3. 完全兼容 ROS 2
基于 ament_cmake 构建。

使用 rclcpp 和 rviz_common 现代 API 重写。

支持通过配置文件修改发布话题（默认为 /goal）。

📺 效果演示
(建议此处放一个 GIF 动图：展示鼠标左键拖动箭头，然后右键拉起高度，最后松开发布的过程)

🛠️ 如何安装与使用
项目已开源，安装非常简单。

1. 准备工作空间
确保你已经安装了 ROS 2 (推荐 Humble 或 Iron 版本)。

Bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
2. 克隆代码
(请将下方的链接替换为您实际上传的 GitHub 仓库链接)

Bash

git clone https://github.com/您的用户名/rviz_3d_nav_goal_tool.git
3. 编译
Bash

cd ~/ros2_ws
colcon build --packages-select rviz_3d_nav_goal_tool
source install/setup.bash
4. 在 RViz2 中加载
启动 RViz2：rviz2

点击顶部菜单栏的 Panels -> Add New Panel。

在列表中找到 rviz_3d_nav_goal_tool，选择 3D Nav Goal 并添加。

现在，你会看到工具栏多了一个红色的图标（或加号图标），选中它即可开始在 3D 空间中“指哪打哪”！

📝 话题配置
默认情况下，插件发布的话题是 /goal。如果你需要修改话题名称（例如改为 /move_base_simple/goal 或 /drone/goal），可以通过 RViz 的属性面板直接修改，无需重新编译代码。

🤝 开源共建
ROS 的生态发展离不开社区的贡献。这个插件的移植过程也趟过了不少 ROS 1 到 ROS 2 API 变迁的“坑”（比如 Ogre 的调用、pluginlib 的宏定义等）。

目前代码已托管在 GitHub，欢迎大家 Star ⭐️、提 Issue 或提交 PR！
