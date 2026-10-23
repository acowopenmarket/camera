# 使用说明

## 环境与离线检查

建议使用源码工作副本，Ubuntu 22.04、Python 3.10、ROS2 Humble、numpy、OpenCV。ROS2 图像节点另需 `cv_bridge`、`message_filters`、`sensor_msgs`。

```bash
python3 -m pip install -e .
python3 -m unittest discover -s tests -v
```

机器人姿态纯数学快照随仓库提供，可通过 `ESTUN_CORE_PATH` 指定已有项目的 `core.py`。外部版本与标定指纹不一致时拒绝计算，不能直接修改指纹绕过。

## 相机中心点

官方 OrbbecSDK_ROS2 的 `v2-main` 安装在独立 `~/orbbec_ws`，自定义位置设置 `ORBBEC_WS`。将 `vision_center/ros2/camera_center_test` 包链接或复制至自己的相机工作空间 `src/`，`colcon build --packages-select camera_center_test` 后启动：

```bash
bash scripts/run_camera.sh
# 另一终端
bash scripts/run_center_test.sh
```

核对节点输出的实际话题、图像尺寸、光学坐标系和深度单位，不能将其他 SDK 配置的坐标直接混用。结束时 Ctrl+C；`stop_camera.sh` 用于关闭本项目 PID 记录对应的相机进程。

## 手动采集与求解

固定 12×9 方格棋盘格（11×8 内部角点、格长20 mm），板和相机安装在采集期间保持固定。设置 `ESTUN_WS` 指向现有 `estun_segmented` 的 ROS2 工作空间；其桥必须支持 `--receive-only`。

先在 Windows 按原机械臂项目说明启动只读反馈转发器，显式设置控制器地址、Windows 反馈接收地址和 WSL 目标地址；本仓库不保存现场 IP。USB 转发按现有 usbipd 配置完成。

```bash
source scripts/handeye_env.sh
# CONTROLLER_IP 和 FEEDBACK_SOURCE_IP 必须由现场配置提供
bash scripts/run_handeye_session.sh --dataset results/handeye_current \
  --controller-ip "$CONTROLLER_IP" --feedback-source-ip "$FEEDBACK_SOURCE_IP"
```

操作者通过现有 Web 界面手动切换姿态。停稳、完整识别棋盘格后按 `s` 保存训练样本，至少15组，包含多个方向的旋转和位置变化；另采至少5组新姿态按 `v` 保存验证样本。验证样本不参与拟合，用来检查换一个观察姿态时坐标是否一致。画面显示保存成功或拒绝原因。

`g` 记录已选定抓取朝向，`p` 保存固定棋盘格的实际深度目标，`q` 退出。采集会检查图像/位姿同步及静止状态；结束后会清理本会话启动的相机和只读桥，外部 Windows 转发器由启动它的终端关闭。

```bash
python3 -m handeye_calibration.cli solve \
  --dataset results/handeye_current --output results/handeye_current/calibration.json
```

检查训练/验证指标及姿态覆盖，不能通过删掉失败验证样本来宣称精度通过。

## 目标计算（只读）

使用同一标定安装和相机内参、捕获时同步 RobotInfo 位姿以及相机光学系下的三维点。实时运行时每个新图像必须重新关联对应位姿，不能以计算时位姿替代拍摄时位姿。

```bash
python3 -m handeye_calibration.cli target \
  --calibration results/handeye_current/calibration.json \
  --observation results/handeye_current/target_observation.json \
  --grasp configs/grasp_down.example.json --point-is-grasp-center \
  --output results/flange_target.json
```

仅在输入点确实是选定夹持中心时使用 `--point-is-grasp-center`；可见表面点需改用 `--grasp-height-base-z` 显式指定基座系夹持高度（米）。夹持高度不等于法兰高度。

输出 `x,y,z`（米）、`rx,ry,rz`（弧度）及兼容字段数组；Z下限失败会明确标记。示例朝向和偏移为本次操作者确认的计算参数，偏移仍需实物验证。程序不发送运动，不代表已通过碰撞、可达性或真实定位验证。

`medical-observation` 对接既有医疗项目三维中心，要求源图、深度几何、单位和同步信息明确；不能直接信任缺少时间戳的旧 HTTP 返回。
