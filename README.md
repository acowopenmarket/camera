# S3-60 Pro + Gemini 335 相机工具

ROS2 Humble / Ubuntu 22.04 下的 Eye-in-Hand 标定、RGB-D 三维点计算和法兰目标位姿计算。当前阶段只读：不会发送 RobotMove 或夹爪控制命令。

| 模块 | 作用 |
| --- | --- |
| `vision_center/` | 对齐 RGB-D 中心点深度、相机内参反投影；对接已有医疗项目的三维中心输出 |
| `handeye_calibration/` | 棋盘格采集、保存反馈、手眼求解、独立验证样本评估 |
| `coordinate_transform/` | 相机点到基座变换、夹爪偏移补偿、RobotMove 六分量生成 |
| `configs/`、`tests/`、`docs/` | 配置示例、离线测试、使用与验证说明 |

## 使用

```bash
python3 -m pip install -e .
python3 -m unittest discover -s tests -v
python3 -m handeye_calibration.cli --help
```

ROS2 取流需另行安装官方 [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) 的 `v2-main` 驱动和现有 `estun_segmented` 只读反馈桥。详见 [使用说明](docs/usage.md)。本仓库不包含相机 SDK、机械臂主程序、夹爪程序或识别模型。

## 当前验证范围

2026-10-23 的真实棋盘格采集使用 15 个训练姿态和 5 个独立验证姿态；验证角点位置 RMS 约 0.660 mm，深度固定点散布 RMS 约 0.562 mm。详见 [验证记录](docs/validation_2026-10-23.md)。这些是样本一致性指标，不代表实际抓取精度或外部量具测量精度。

向下抓取姿态按 `[-π, 0, 0] rad`，夹爪偏移暂按法兰局部 `[0, 0, 0.15384] m`；实际夹持中心尚待核对。法兰目标 `z < 0.201 m` 时拒绝标记为可执行，绝不截断目标高度。所有输出保持 `executable=false` 和 `motion_sent=false`。

现场图像、原始采集记录、日志、模型和原医疗项目留在本地并由 `.gitignore` 排除。仓库中的标定示例仅适用于原安装条件，不能直接作为其他安装的已验证标定结果。
