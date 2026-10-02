# ProBridge

A module for providing transport of ROS messages bypassing DDS.
Collects messages on one machine, sends them via 0MQ to machines from a list of addresses, on the other side receives messages and publishes them back to ROS.

- [Dependencies](#1-dependencies)
- [Usage](#2-usage)
- [Config example](#3-config-example)
- [Advanced params](#4-advanced-params)
  - [qos](#41-qos)
  - [rate](#42-rate)
  - [compression_level](#43-compression_level)
  - [latch](#44-latch)
- [Services](#5-services)

### 1. Dependencies
  - CycloneDDS is required for communication between 
    - [UnityBridge](https://github.com/RosProBridge/UnityBridge)->ROS2
    - ROS2->[UnityBridge](https://github.com/RosProBridge/UnityBridge)
    - ROS1->ROS2

    > For communication ROS2->ROS2, it does not matter which RMW implementation is used, as long as the same RMW implementation is used on both machines.
 
    For installation follow the official [ROS documentation](https://docs.ros.org/en/humble/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html)

### 2. Usage

  ### 2.1 Installation

  ##### 2.1.1 ROS2

  ```bash
  user@~: mkdir -p ros_ws/src
  user@~: cd ros_ws/src
  user@ros_ws/src: git clone https://github.com/RosProBridge/RosProBridge.git
  user@ros_ws: cd ..
  user@ros_ws: rosdep install --from-paths src --ignore-src -r
  user@ros_ws: colcon build
  ```

  #### 2.1.2 ROS1

  ```bash
  user@user~:$ mkdir -p ros_ws/src
  user@user:$ cd ros_ws
  user@user:ros_ws$ catkin init
  user@user:ros_ws$ cd src/
  user@user:ros_ws/src$ git clone https://github.com/RosProBridge/RosProBridge.git
  user@user:ros_ws$ cd ..
  user@user:ros_ws$ rosdep install --from-paths src --ignore-src -r
  user@user:ros_ws$ catkin build
  ```

  ### 2.2 Starting the node

  Once pro_bridge is installed, you can start the node using the following commands:

  #### 2.2.1 ROS2 

    ```bash
    user@user:ros_ws$ source install/setup.bash
    user@user:ros_ws$ ros2 run pro_bridge bridge.py src/RosProBridge/config/recv.json
    ```

  #### 2.2.2 ROS1

  ```bash
  user@user:ros_ws$ source devel/setup.bash
  user@user:ros_ws$ rosrun pro_bridge bridge.py src/RosProBridge/config/recv.json
  ```


### 3. Config Example
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

### 4. Advanced params

#### 4.1 qos

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

#### 4.2 rate

```
"rate": 10
```

Target posting frequency

#### 4.3 compression_level:

```
"compression_level": 0 # range[0:9]
```

If the value is 0, there will be no compression; otherwise, the ROS message will be compressed

#### 4.4 latch:

```
"latch": false
```

If the value is true, last received ROS message will be sent to newly connected clients

### 5. Services

> ROS2 only for now; ROS1 ignores services with a warning.

Services work in both directions, like topics:

- **Served by the remote side** (e.g. [UnityBridge](https://github.com/RosProBridge/UnityBridge) `ProBridgeService`), called from ROS: nothing to configure. The remote side advertises the service (on start and on every connection); the bridge creates it in ROS on the first advertisement, like a publisher on the first message. The advertisement carries the port of the remote server (`"p"`), the bridge takes the remote IP from the connection and sends the calls there, so the remote side does not have to be in `hosts`.
- **Served in ROS**, called by the remote side (UnityBridge `ProBridgeServiceClient`): list the service in `services` of a `published` group, as topics sent to the remote side. The response goes back to the caller (same `"p"` + connection IP); `hosts` of the group are used only if the request has no `"p"`.

```json
{
  "id": "sim",
  "host": "0.0.0.0:47778",
  "published": [
    {
      "hosts": [],
      "topics": [],
      "services": [
        {
          "name": "/map/save",
          "type": "std_srvs.srv.Trigger",
          "timeout": 5.0
        }
      ]
    }
  ]
}
```

- `name`, `type`: service name and type (`<package>.srv.<Service>`).
- `timeout`: seconds to wait for the ROS service, default `5.0`.
- `compression_level`: compression of the response, as for topics.

A call that fails in the bridge (remote side not connected, no response within 5 s, ROS service not available or not listed in the config) gets a default response with `success = false` and the reason in `message`, if the response has these fields (std_srvs/Trigger, SetBool and alike).

Examples with the services of [UnityBridge](https://github.com/RosProBridge/UnityBridge) (Scene Reload, Sim Pause):

```bash
ros2 service call /sim/reload std_srvs/srv/Trigger
ros2 service call /sim/pause std_srvs/srv/SetBool "{data: true}"    # pause
ros2 service call /sim/pause std_srvs/srv/SetBool "{data: false}"   # resume
```

Protocol: service messages use the message header with extra fields: the kind `"k"` (`"adv"` advertisement, `"req"` request, `"res"` response), the call `"id"` (a response carries the id of its request) and, in advertisements and requests, `"p"` - the port to answer to at the sender's IP. `n` is the service name, `t` the service type, the payload is the CDR-serialized `<Service>_Request` / `<Service>_Response`. Calls run in a reentrant callback group on a multi-threaded executor, so a pending call blocks neither topics nor other calls.
