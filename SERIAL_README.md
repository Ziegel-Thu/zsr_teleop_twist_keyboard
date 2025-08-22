# ZSR 串口发布器使用说明

这个程序可以订阅 `zsr_cmd_vel` 话题，并将接收到的 Twist 消息通过串口发送出去。

## 功能特性

- 订阅 ROS2 的 `zsr_cmd_vel` 话题
- 通过串口发送数据到硬件设备
- 支持自定义串口参数（端口、波特率等）
- 自动重连机制
- 数据包格式包含校验和
- 线程安全的数据发送

## 数据包格式

支持两种数据格式：

### 1. 文本格式 (data_format=text)
```
ZS,linear_x,linear_y,angular_z\n
```
- 简单易读，适合调试
- 例如：`ZS,1.500,0.000,0.750\n`

### 2. 二进制格式 (data_format=binary)
```
[ZS][linear_x(4字节)][linear_y(4字节)][angular_z(4字节)][timestamp(4字节)][checksum(2字节)]
```

- `ZS`: 包头标识（2字节）
- `linear_x`: X方向线速度（4字节浮点数，大端序）
- `linear_y`: Y方向线速度（4字节浮点数，大端序）
- `angular_z`: Z方向角速度（4字节浮点数，大端序）
- `timestamp`: 时间戳（4字节无符号整数，毫秒）
- `checksum`: 校验和（2字节CRC16或1字节简单校验和）

### 数据范围限制
- 线速度：±10 m/s
- 角速度：±5 rad/s

## 安装依赖

确保已安装 `pyserial` 库：
```bash
pip install pyserial
```

## 使用方法

### 方法1：直接运行节点

```bash
# 使用默认参数
ros2 run zsr_teleop_twist_keyboard zsr_serial_publisher

# 指定串口参数
ros2 run zsr_teleop_twist_keyboard zsr_serial_publisher --ros-args \
    -p port:=/dev/ttyUSB1 \
    -p baudrate:=9600 \
    -p topic_name:=cmd_vel \
    -p data_format:=text \
    -p use_crc:=false
```

### 方法2：使用启动文件

```bash
# 使用默认参数
ros2 launch zsr_teleop_twist_keyboard launch_serial_publisher.launch.py

# 指定参数
ros2 launch zsr_teleop_twist_keyboard launch_serial_publisher.launch.py \
    port:=/dev/ttyUSB1 \
    baudrate:=9600 \
    topic_name:=cmd_vel \
    data_format:=text \
    use_crc:=false
```

## 参数说明

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `port` | `/dev/ttyUSB0` | 串口设备路径 |
| `baudrate` | `115200` | 串口波特率 |
| `timeout` | `1.0` | 串口超时时间（秒） |
| `topic_name` | `zsr_cmd_vel` | 订阅的话题名称 |
| `data_format` | `binary` | 数据格式：`binary` 或 `text` |
| `use_crc` | `true` | 是否使用CRC16校验（仅二进制格式） |

## 测试方法

1. 启动串口发布器：
```bash
ros2 run zsr_teleop_twist_keyboard zsr_serial_publisher
```

2. 在另一个终端发送测试消息：
```bash
ros2 topic pub /zsr_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

3. 使用串口调试工具查看发送的数据：
```bash
# 在Linux上使用screen
screen /dev/ttyUSB0 115200

# 或使用minicom
minicom -D /dev/ttyUSB0 -b 115200
```

## 故障排除

### 串口权限问题
如果遇到权限错误，需要将用户添加到dialout组：
```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 串口设备不存在
检查串口设备是否存在：
```bash
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*
```

### 波特率不匹配
确保串口波特率与硬件设备匹配。

## 代码结构

- `ZSRSerialPublisher`: 主要的ROS2节点类
- `init_serial()`: 初始化串口连接
- `cmd_vel_callback()`: 处理接收到的Twist消息
- `check_serial_connection()`: 定期检查串口连接状态
- `send_custom_command()`: 发送自定义命令（可选功能）

## 注意事项

1. 确保串口设备已正确连接
2. 检查串口权限设置
3. 确保波特率与硬件设备匹配
4. 程序会自动尝试重连断开的串口
5. 数据发送是线程安全的 