# 清扫机构 ROS 话题说明（dlrobot_robot）

## 话题方向

| 话题 | 方向 | 作用 |
|------|------|------|
| `/clean/cmd/control` | App → 底盘 | **控制命令**（希望开/关哪些机构） |
| `/robot_state/clean_system_android` | 底盘 → App | **状态回显**（当前各机构开/关） |
| `eva/robot_state` | 底盘 → App | 电量、急停等整机状态 |

App 点击车灯、中刷等时，http_server 向 `/clean/cmd/control` **发布**；`dlrobot_robot` **订阅**并在 `gpioWriteCallback` 中处理，再向 `/robot_state/clean_system_android` **发布**当前状态供界面刷新。

## 消息类型

`skyee_msg/clean_system`：见 `msgs/skyee_msg/msg/clean_system.msg`

- `switch_control`：清扫系统总开关（一键启停核心机构）
- `clean_model`：力度模式（由参数 `~clean_model` 与状态回显中的 `clean_model` 字段表示）
- 其余字段：边刷、中刷、风机、水泵、灯等

## 下行串口帧 `tx[1]`（ROS → 下位机）

11 字节发送帧中 `tx[1]` 为**底盘命令字节**（原 Wheeltec 预留），与上行 `rx[1]` 状态字节对应：

| bit | 宏 | `clean_system.msg` 字段 |
|-----|-----|-------------------------|
| 0 | `CHASSIS_CMD_BIT_CLEAN_MODEL` | clean_model |
| 1 | `CHASSIS_CMD_BIT_SWITCH_CONTROL` | switch_control |
| 2 | `CHASSIS_CMD_BIT_BESIDE_BRUSH` | beside_brush |
| 3 | `CHASSIS_CMD_BIT_MIDDLE_BRUSH` | middle_brush |
| 4 | `CHASSIS_CMD_BIT_PUSHBEAM` | pushbeam |
| 5 | `CHASSIS_CMD_BIT_FAN` | fan |
| 6 | `CHASSIS_CMD_BIT_WATER_PUMP` | water_pump |
| 7 | `CHASSIS_CMD_BIT_VIBRATING_DUST` | vibrating_dust |

第 9、10 字段 `baffle`、`lamp` 超出 1 字节，可后续用 `tx[2]` 扩展。

- 发速度时：`Cmd_Vel_Callback` → `WriteMotionFrameToMcu()` 自动带上当前 `tx[1]`
- 仅改清扫机构时：`gpioWriteCallback` → `ApplyCleanSystemHardware()` 以 0 速度再发一帧，更新 `tx[1]`

下位机固件需按上表解析；若位定义不同，只改 [`dlrobot_robot.h`](../include/dlrobot_robot.h) 中宏即可。

## 实现位置

- 订阅命令：[`dlrobot_robot.cpp`](../src/dlrobot_robot.cpp) `gpioWriteCallback`
- 发布状态：`PublishCleanSystemAndroid()`
- 串口下发：`BuildChassisCommandTxByte()` + `WriteMotionFrameToMcu()`
