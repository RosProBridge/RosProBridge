# ProBridge

A module for providing transport of ROS messages bypassing DDS.
Collects messages on one machine, sends them via 0MQ to machines from a list of addresses, on the other side receives messages and publishes them back to ROS.

- [Config example](#1-config-example)
- [Advanced params](#2-advanced-params)
  - [qos](#21-qos)
  - [rate](#22-rate)
  - [compression_level](#23-compression_level)
  - [latch](#24-latch)

### 1. Config Example
----
Config example:

```json
{
  "id": "some_id_value",
  "host": "0.0.0.0:47777",
  "published": [
    {
      "hosts":["192.168.1.1:47777"],
        {
          "name": "/lidar/fl/rslidar_points",
          "type": "sensor_msgs.msg.PointCloud2",
          "qos": 10,
          "compression_level": 1
        },
    }
  ]
}
```

### 2. Advanced params

#### 2.1 qos

qos value may be as int
```
  "qos": 100
  "qos": 200
  ...
```

as string:
```
  "qos": "qos_profile_parameters"
  "qos": "qos_profile_sensor_data"
  "qos": "qos_profile_services_default"
  "qos": "qos_profile_system_default"
  "qos": "qos_profile_unknown"
```

as dict:
```json
  "qos" {
    "reliability": "BEST_EFFORT", # SYSTEM_DEFAULT | BEST_EFFORT | RELIABLE | UNKNOWN
    "history": "KEEP_LAST", # KEEP_ALL | KEEP_LAST | SYSTEM_DEFAULT | UNKNOWN
    "depth": 1,
    "durability": "VOLATILE", # SYSTEM_DEFAULT | TRANSIENT_LOCAL | VOLATILE | UNKNOWN
    "liveliness": "AUTOMATIC" # SYSTEM_DEFAULT | AUTOMATIC | MANUAL_BY_TOPIC | UNKNOWN
  }
```

Ensure that the "qos" parameter can be used to allow publishing messages from ROS1 to ROS2 with the specified QoS settings.

#### 2.2 rate

```
"rate": 10
```

Target posting frequency

#### 2.3 compression_level:

```
"compression_level": 0 # range[0:9]
```

If the value is 0, there will be no compression; otherwise, the ROS message will be compressed

#### 2.4 latch:

```
"latch": false
```

If the value is true, last received ROS message will be sent to newly connected clients