# PI05 Isaac Sim 仿真场景

在 Isaac Sim 6.0 中使用 **File → Open** 打开同目录的 `workspace_pi05.usda`。如果出现 **ScriptNode Warning**，在当前会话授权后点 **Play**。内嵌 Action Graph 会启用重力补偿和阻尼，无需运行独立控制脚本。

场景包含天机双臂、舞肌双手、相机、桌面和三面围挡。初始关节角来自实机双臂读数，手指采用模型的初始姿态。

## 场景文件

仓库有 6 个 `.usda` 图层。查看三路相机时打开 [`workspace_pi05_cameras.usda`](workspace_pi05_cameras.usda)，查看机器人仿真时可以直接打开 [`workspace_pi05.usda`](workspace_pi05.usda)。入口会自动载入表格中位于其下方的图层。

| 文件 | 用途 |
| --- | --- |
| [`workspace_pi05_cameras.usda`](workspace_pi05_cameras.usda) | 加载完整仿真并附加头部、左腕和右腕的 RGB 相机。 |
| [`workspace_pi05.usda`](workspace_pi05.usda) | 场景入口。内嵌 Action Graph，播放时启用重力补偿和阻尼。 |
| [`workspace_pi05_impedance.usda`](workspace_pi05_impedance.usda) | 定义 PI05 对应的关节刚度与末端负载质量和质心。 |
| [`workspace_measured_arms.usda`](workspace_measured_arms.usda) | 保存实测的双臂初始关节角、驱动目标和对应的连杆姿态。 |
| [`workspace_factory_robot.usda`](workspace_factory_robot.usda) | 装配舞肌双手，定义机械臂的质量、惯量、关节限位和碰撞形状。 |
| [`workspace_appearance.usda`](workspace_appearance.usda) | 定义场景材质，并载入机器人、桌面和围挡的基础场景。 |

基础场景保存在 [`robot_workspace_wall_aligned_full_column.usdc`](robot_workspace_wall_aligned_full_column.usdc)。双手装配引用 `official_wuji/` 里的模型。移动或复制场景时请保留整个目录结构，图层和模型通过相对路径引用。

## RGB 相机预览

打开相机场景后，从视口的 **Perspective** 菜单选择头部或腕部相机。头部相机固定在机架上，双腕相机随各自的末端连杆运动。光心和朝向来自安装关系，焦距、主点和畸变参数来自 [ROS 驱动的 CameraInfo](camera_rgb_driver_info.json)。

下图为双臂初始姿态下的仿真画面。头部原图为 1280×720，双腕原图均为 848×480。[头部](camera_preview/head.png) · [左腕](camera_preview/left_wrist.png) · [右腕](camera_preview/right_wrist.png)

![上方为头部画面，下方依次为左腕和右腕画面](camera_preview/rgb_camera_views.jpg)

三路相机都使用 4 cm 的近裁剪距离，以避开 CAD 相机外壳造成的遮挡。光心前 4 cm 内的物体会被裁掉。

## 控制与来源

右臂使用 PI05 部署的刚度、阻尼比例和末端质量与质心。左臂使用同样的阻抗比例及左臂自己的负载配置。阻尼根据初始姿态下的各关节惯量近似计算，末端惯量采用等效质量近似。场景仅执行仿真控制，不连接实机或运行策略推理。

`build_pi05_scene.py` 使用当前装配与初始位图层生成两个 PI05 USD 图层，并将 `pi05_graph_controller.py` 的源码嵌入场景入口。舞肌手模型来自 [Wuji Technology](https://github.com/wuji-technology/wuji-description/tree/4feaeb69bac8e06325b090abfbb3d784a5a9194a)，授权文件位于 `official_wuji/LICENSE`。
