# PI05 Isaac Sim 仿真场景

在 Isaac Sim 6.0 中使用 **File → Open** 打开同目录的 `workspace_pi05.usda`。如果出现 **ScriptNode Warning**，在当前会话授权后点 **Play**。内嵌 Action Graph 会启用重力补偿和阻尼，无需运行独立控制脚本。

场景包含天机双臂、舞肌双手、相机、桌面和三面围挡。初始关节角来自实机双臂读数，手指采用模型的初始姿态。

## 场景文件

仓库有 5 个 `.usda` 图层。在 Isaac Sim 中打开最上层的 [`workspace_pi05.usda`](workspace_pi05.usda)，其余图层会按表格顺序自动载入。

| 文件 | 用途 |
| --- | --- |
| [`workspace_pi05.usda`](workspace_pi05.usda) | 场景入口。内嵌 Action Graph，播放时启用重力补偿和阻尼。 |
| [`workspace_pi05_impedance.usda`](workspace_pi05_impedance.usda) | 定义 PI05 对应的关节刚度与末端负载质量和质心。 |
| [`workspace_measured_arms.usda`](workspace_measured_arms.usda) | 保存实测的双臂初始关节角、驱动目标和对应的连杆姿态。 |
| [`workspace_factory_robot.usda`](workspace_factory_robot.usda) | 装配舞肌双手，定义机械臂的质量、惯量、关节限位和碰撞形状。 |
| [`workspace_appearance.usda`](workspace_appearance.usda) | 定义场景材质，并载入机器人、桌面和围挡的基础场景。 |

基础场景保存在 [`robot_workspace_wall_aligned_full_column.usdc`](robot_workspace_wall_aligned_full_column.usdc)。双手装配引用 `official_wuji/` 里的模型。移动或复制场景时请保留整个目录结构，图层和模型通过相对路径引用。

## 控制与来源

右臂使用 PI05 部署的刚度、阻尼比例和末端质量与质心。左臂使用同样的阻抗比例及左臂自己的负载配置。阻尼根据初始姿态下的各关节惯量近似计算，末端惯量采用等效质量近似。场景仅执行仿真控制，不连接实机或运行策略推理。

`build_pi05_scene.py` 使用当前装配与初始位图层生成两个 PI05 USD 图层，并将 `pi05_graph_controller.py` 的源码嵌入场景入口。舞肌手模型来自 [Wuji Technology](https://github.com/wuji-technology/wuji-description/tree/4feaeb69bac8e06325b090abfbb3d784a5a9194a)，授权文件位于 `official_wuji/LICENSE`。
