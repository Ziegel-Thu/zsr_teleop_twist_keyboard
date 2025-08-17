# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import threading

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def twist_to_channels(twist):
    """
    将 twist 消息转换为 16 个 channel 的信号
    
    Args:
        twist: geometry_msgs.msg.Twist 消息
        
    Returns:
        list: 16个channel的信号值，范围0-2047，默认都是1024（中间值）
    """
    # 初始化16个channel，默认值为1024（中间值）
    channels = [1024] * 16
    def map_to_range(value, min_val=-2.0, max_val=2.0, output_min=0, output_max=2047):
        normalized = (value - min_val) / (max_val - min_val)
        return int(normalized * (output_max - output_min) + output_min)
    
    channels[3] = map_to_range(twist.linear.x)
    channels[2] = map_to_range(-twist.linear.y)
    channels[0] = map_to_range(-twist.angular.z)
    
    return channels


def channels_to_sbus(channels):
    """
    将16个channel值转换为SBUS格式
    
    Args:
        channels: list of 16 integers, 每个值范围0-2047 (11位)
        
    Returns:
        bytes: 25字节的SBUS数据帧
    """

    for i in range(16):
        channels[i] = max(0, min(2047, channels[i]))
    
    sbus_frame = bytearray(25)

    sbus_frame[0] = 0x0F
    
    bit_pos = 0
    byte_pos = 1
    
    for channel in channels:
        channel_11bit = channel & 0x7FF
        
        for bit in range(11):
            if bit_pos >= 8:
                bit_pos = 0
                byte_pos += 1
            
            if channel_11bit & (1 << bit):
                sbus_frame[byte_pos] |= (1 << bit_pos)
            
            bit_pos += 1
    
    # 标志位 (通常为0)
    sbus_frame[23] = 0x00
    
    # 结束字节
    sbus_frame[24] = 0x00
    
    return bytes(sbus_frame)


def twist_to_sbus(twist):
    """
    直接将twist转换为SBUS格式的便捷函数
    
    Args:
        twist: geometry_msgs.msg.Twist 消息
        
    Returns:
        bytes: 25字节的SBUS数据帧
    """
    channels = twist_to_channels(twist)
    return channels_to_sbus(channels)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('zsr_teleop_twist_keyboard')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    # 改为发布SBUS数据
    from std_msgs.msg import ByteMultiArray
    pub = node.create_publisher(ByteMultiArray, 'zsr_cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            
            # 只改这里：转换为SBUS并发布
            sbus_data = twist_to_sbus(twist)
            sbus_msg = ByteMultiArray()
            sbus_msg.data = list(sbus_data)
            pub.publish(sbus_msg)

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # 这里也改为发布SBUS
        sbus_data = twist_to_sbus(twist)
        sbus_msg = ByteMultiArray()
        sbus_msg.data = list(sbus_data)
        pub.publish(sbus_msg)
        
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
