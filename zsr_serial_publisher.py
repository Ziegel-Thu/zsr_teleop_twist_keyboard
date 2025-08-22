#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray  # 改为订阅 UInt8MultiArray
import serial
import struct
import time
import threading
from typing import Optional


class ZSRSerialPublisher(Node):
    """
    订阅zsr_cmd_vel话题并通过串口发送数据的节点
    """
    
    def __init__(self):
        super().__init__('zsr_serial_publisher')
        
        # 串口配置参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('topic_name', 'zsr_cmd_vel')
        self.declare_parameter('data_format', 'binary')  # 'binary' 或 'text'
        self.declare_parameter('use_crc', True)  # 是否使用CRC校验
        
        # 获取参数
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.topic_name = self.get_parameter('topic_name').value
        self.data_format = self.get_parameter('data_format').value
        self.use_crc = self.get_parameter('use_crc').value
        
        # 串口对象
        self.serial_port: Optional[serial.Serial] = None
        
        # 创建订阅者 - 改为订阅 UInt8MultiArray
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            self.topic_name,
            self.sbus_callback,  # 改为新的回调函数名
            10
        )
        
        # 创建定时器用于定期检查串口连接
        self.timer = self.create_timer(5.0, self.check_serial_connection)
        
        # 线程锁
        self.serial_lock = threading.Lock()
        
        # 初始化串口连接
        self.init_serial()
        
        self.get_logger().info(f'ZSR串口发布器已启动，订阅话题: {self.topic_name}')
        self.get_logger().info(f'串口配置: {self.port}, 波特率: {self.baudrate}')
    
    def init_serial(self):
        """初始化串口连接"""
        try:
            if self.serial_port is not None:
                self.serial_port.close()
            
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            if self.serial_port.is_open:
                self.get_logger().info(f'串口 {self.port} 连接成功')
            else:
                self.get_logger().error(f'串口 {self.port} 连接失败')
                
        except serial.SerialException as e:
            self.get_logger().error(f'串口连接错误: {str(e)}')
            self.serial_port = None
        except Exception as e:
            self.get_logger().error(f'初始化串口时发生错误: {str(e)}')
            self.serial_port = None
    
    def check_serial_connection(self):
        """检查串口连接状态"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn('串口连接已断开，尝试重新连接...')
            self.init_serial()
    
    def sbus_callback(self, msg: UInt8MultiArray):
        """
        处理接收到的SBUS消息并通过串口发送
        
        Args:
            msg: 接收到的UInt8MultiArray消息（SBUS数据）
        """
        # 打印接收到的数据
        self.get_logger().info(f'收到SBUS消息: 数据长度={len(msg.data)}')
        self.get_logger().debug(f'SBUS数据: {[hex(x) for x in msg.data[:10]]}...')  # 只显示前10个字节
        
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn('串口未连接，无法发送数据')
            return
        
        try:
            # 直接发送SBUS数据
            sbus_data = bytes(msg.data)
            
            # 发送数据
            with self.serial_lock:
                self.serial_port.write(sbus_data)
                self.serial_port.flush()
            
            # 记录发送的数据
            self.get_logger().debug(f'发送SBUS数据: {len(sbus_data)} 字节')
            
        except serial.SerialException as e:
            self.get_logger().error(f'串口发送错误: {str(e)}')
            self.serial_port = None
        except Exception as e:
            self.get_logger().error(f'处理消息时发生错误: {str(e)}')
    
    def create_text_packet(self, linear_x: float, linear_y: float, angular_z: float) -> bytes:
        """
        创建文本格式的数据包
        
        Args:
            linear_x: X方向线速度
            linear_y: Y方向线速度
            angular_z: Z方向角速度
            
        Returns:
            文本格式的数据包
        """
        # 格式: "ZS,linear_x,linear_y,angular_z\n"
        text = f"ZS,{linear_x:.3f},{linear_y:.3f},{angular_z:.3f}\n"
        return text.encode('utf-8')
    
    def create_binary_packet(self, linear_x: float, linear_y: float, angular_z: float) -> bytes:
        """
        创建二进制格式的数据包
        
        Args:
            linear_x: X方向线速度
            linear_y: Y方向线速度
            angular_z: Z方向角速度
            
        Returns:
            二进制格式的数据包
        """
        # 格式: [header(2字节)] [linear_x(4字节)] [linear_y(4字节)] [angular_z(4字节)] [timestamp(4字节)] [checksum(2字节)]
        header = b'ZS'  # 包头标识
        
        # 使用大端序 (网络字节序) 确保跨平台兼容性
        linear_x_bytes = struct.pack('>f', linear_x)
        linear_y_bytes = struct.pack('>f', linear_y)
        angular_z_bytes = struct.pack('>f', angular_z)
        
        # 添加时间戳 (毫秒)
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        timestamp_bytes = struct.pack('>I', timestamp)
        
        if self.use_crc:
            # 使用CRC16校验和
            data_bytes = linear_x_bytes + linear_y_bytes + angular_z_bytes + timestamp_bytes
            checksum = self.calculate_crc16(data_bytes)
            checksum_bytes = struct.pack('>H', checksum)
            packet = header + data_bytes + checksum_bytes
        else:
            # 使用简单校验和
            data_bytes = linear_x_bytes + linear_y_bytes + angular_z_bytes + timestamp_bytes
            checksum = sum(data_bytes) & 0xFF
            checksum_bytes = struct.pack('B', checksum)
            packet = header + data_bytes + checksum_bytes
        
        return packet
    
    def calculate_crc16(self, data: bytes) -> int:
        """
        计算CRC16校验和
        
        Args:
            data: 要计算校验和的数据
            
        Returns:
            CRC16校验和值
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc &= 0xFFFF
        return crc
    
    def send_custom_command(self, command: str):
        """
        发送自定义命令
        
        Args:
            command: 要发送的命令字符串
        """
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn('串口未连接，无法发送命令')
            return
        
        try:
            with self.serial_lock:
                self.serial_port.write(command.encode('utf-8'))
                self.serial_port.flush()
            
            self.get_logger().info(f'发送命令: {command}')
            
        except serial.SerialException as e:
            self.get_logger().error(f'发送命令时串口错误: {str(e)}')
            self.serial_port = None
        except Exception as e:
            self.get_logger().error(f'发送命令时发生错误: {str(e)}')
    
    def on_shutdown(self):
        """节点关闭时的清理工作"""
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('串口连接已关闭')


def main(args=None):
    rclpy.init(args=args)
    
    node = ZSRSerialPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
