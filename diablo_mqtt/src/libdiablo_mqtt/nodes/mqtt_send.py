import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState, NavSatFix

def mqtt_callback(msg):
    # 解析收到的消息
    received_data = msg.data.split()  # 假设消息格式为 "setvel 线速度 角速度"
    cmd_type = received_data[0]
    # 假设消息格式为 
    '''
    receive: "setvel linear_velocity angular_velocity" , such as setvel 1.0 0.0
             "setled led_num r g b"              , such as setled 0 255 255 255
              
             |---|---|
           / ^   ^   ^ \
          ^             ^
          0  1   2   3  0

          led_num : 
            0: illumination_led
            1: right_led
            2: middle_led
            3: right_led
    description: prase and convert msgs from mqtt and publish to ROS topic boardcast
    return 
    '''
    # 检查消息格式
    
    if len(received_data) > 5:
        print("Invalid message format or command")
        return
    else:
        # 处理 set_vel 命令
        if cmd_type == 'setvel':
            try:
                linear_vel = float(received_data[1])
                angular_vel = float(received_data[2])
            except ValueError:
                print("Invalid velocity value")
                return

            # 创建 Twist 消息
            twist_msg = Twist()
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = angular_vel

            # 发布到 cmd_vel 话题
            cmd_vel_pub.publish(twist_msg)

        # 处理 set_led 命令
        elif cmd_type == 'setled':
            try:
                # 灯的序号
                led_data = String()
                led_data.data = received_data[1] + ' ' + received_data[2] + ' ' + received_data[3] + ' ' + received_data[4]

            except ValueError:
                print("Invalid LED color value or LED num")
                return

            # 发布到相应的 LED 话题
            led_pub.publish(led_data)

        elif cmd_type == 'setpose':
            try:
                # True
                stand_data = String()
                if received_data[1] == '1':
                    stand_data.data = 'standup'
                elif received_data[1] == '0':
                    stand_data.data = 'sitdown'
                else:
                    stand_data.data = ''

            except ValueError:
                print("Invalid LED color value or LED num")
                return

            # 发布到相应的 POSE 话题
            stand_pub.publish(stand_data)
        else:
            rclpy.Node.get_logger().warning("Invalid command, please check mqtt publisher")
    

def battery_callback(msg):
    # 获取电池电压信息
    battery_voltage = msg.voltage
    mqtt_msg = String()

    # 构建发送给 mqtt_rxd 话题的消息
    mqtt_msg.data = "BATT " + str(round(battery_voltage, 3))

    # 发布到 mqtt_rxd 话题
    mqtt_rxd_pub.publish(mqtt_msg)

def gps_callback(msg):
    # 获取 GPS 位置信息
    latitude = msg.latitude
    longitude = msg.longitude

    mqtt_msg = String()
    # 构建发送给 mqtt_rxd 话题的消息
    mqtt_msg.data = "GPS {} {}".format(longitude, latitude)

    # 发布到 mqtt_rxd 话题
    mqtt_rxd_pub.publish(mqtt_msg)

def led_callback(msg):
    # 获取 LED 状态信息
    led_data = msg.data.split()
    led_num = led_data[0]
    led_state = int(led_data[1]) + int(led_data[2]) + int(led_data[3])

    mqtt_msg = String()
    # 构建发送给 mqtt_rxd 话题的消息
    mqtt_msg.data = "LED {} {}".format(led_num, led_state)

    # 发布到 mqtt_rxd 话题
    mqtt_rxd_pub.publish(mqtt_msg)

def bodystate_callback(msg):
    # 获取 GPS 位置信息
    latitude = msg.latitude
    longitude = msg.longitude

    mqtt_msg = String()
    # 构建发送给 mqtt_rxd 话题的消息
    mqtt_msg.data = "GPS {} {}".format(longitude, latitude)

    # 发布到 mqtt_rxd 话题
    mqtt_rxd_pub.publish(mqtt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('mqtt_listener')
    subscriber = node.create_subscription(String, 'mqtt_txd', mqtt_callback, 10)
    battery_subscriber = node.create_subscription(BatteryState, '/diablo/sensor/Battery', battery_callback, 10)
    gps_subscriber = node.create_subscription(NavSatFix, '/fix', gps_callback, 10)
    led_subscriber = node.create_subscription(String, '/led', led_callback, 10)
    global stand_pub, led_pub, cmd_vel_pub, mqtt_rxd_pub
    stand_pub = node.create_publisher(String, 'stand', 2)
    led_pub = node.create_publisher(String, 'led', 2)
    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    mqtt_rxd_pub = node.create_publisher(String, '/mqtt_rxd', 10)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
