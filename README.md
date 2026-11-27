# PI05 Isaac Sim 仿真场景

在 Isaac Sim 6.0 中使用 **File → Open** 打开同目录的 `workspace_pi05.usda`。如果出现 **ScriptNode Warning**，在当前会话授权后点 **Play**。内嵌 Action Graph 会启用重力补偿和阻尼，无需运行独立控制脚本。

场景包含天机双臂、舞肌双手、相机、桌面和三面围挡。初始关节角来自实机双臂读数，手指采用模型的初始姿态。

右臂使用 PI05 部署的刚度、阻尼比例和末端质量与质心。左臂使用同样的阻抗比例及左臂自己的负载配置。阻尼根据初始姿态下的各关节惯量近似计算，末端惯量采用等效质量近似。场景仅执行仿真控制，不连接实机或运行策略推理。

`build_pi05_scene.py` 使用当前装配与初始位图层生成两个 PI05 USD 图层，并将 `pi05_graph_controller.py` 的源码嵌入场景入口。舞肌手模型来自 [Wuji Technology](https://github.com/wuji-technology/wuji-description/tree/4feaeb69bac8e06325b090abfbb3d784a5a9194a)，授权文件位于 `official_wuji/LICENSE`。
