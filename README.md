# ProBridge

Модуль для обеспечения транспорта сообщений ROS в обход DDS (может пригодиться для интеграции с системами без ROS и работы ROS через беспроводную связь), собирает сообщения на одной машине, отправляет через UDP на 
машины из списка адресов, с другой стороны получает сообщения и публикует их в обратно в ROS.

----
Пример конфига:
```json
{
  "id": "some_id_value",
  "host": "0.0.0.0:47777",
  "published": [
    {
      "hosts":["192.168.1.1:47777"],
      "topics":[
        {
          "name": "/odometry/imu",
          "type": "sensor_msgs.msg.Imu",
          "qos": "qos_profile_sensor_data"
        },
        {
          "name": "/odometry/nav_sat",
          "type": "nav_msgs.msg.Path",
          "qos": "qos_profile_system_default"
        }
      ]
    },
    {
      "hosts":["192.168.1.1:47777"],
      "topics":[
        {
          "name": "/odometry/odom",
          "type": "nav_msgs.msg.Odometry",
          "qos": "qos_profile_sensor_data"
        }
      ]
    }
  ]
}
```
