# 坐标转换与目标位姿

`geometry.py` 提供 `camera_to_base`、夹爪补偿及 `target_components`；`target.py` 检查观测时间、光学坐标系、内参、姿态约定和标定状态。

使用列向量：`P_base = T_base_flange @ T_flange_camera @ P_camera`。
选定抓取旋转 `R` 后，法兰位置为 `P_grasp_base - R @ [0,0,0.15384]`。长度为米、角度为弧度，顺序为 `x,y,z,rx,ry,rz`。

`estun_math.py` 复用已有项目的纯数学与字段序列化实现；不包含网络发送。来源与完整性记录见 `math_provenance.json` 和 [来源说明](../docs/provenance.md)。
