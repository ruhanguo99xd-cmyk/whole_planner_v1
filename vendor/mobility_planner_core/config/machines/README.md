# 机型参数文件说明

每个机型一个 YAML 文件：`M001.yaml` ... `M007.yaml`

统一字段：
- model_id, description
- lidar(host/port)
- encoder(endpoint/db_address/sample_hz/resolution_ticks_per_rev)
- extrinsic(tx/ty/tz/roll/pitch/yaw)
- mapping(resolution/width/height/origin)
- control(wheel_base/max_speed)
- rtk(host/port/origin_lat/origin_lon/origin_alt)

切换机型：
```bash
python3 scripts/select_machine_profile.py --machine M00X
```
