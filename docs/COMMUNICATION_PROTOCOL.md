# Communication protocol

V3 contains two transport families recovered from the deployed project.

## 1. Canonical point-cloud / LoRa / odometry transport

The documented V3 runtime uses separate TCP endpoints for the three data types. Port numbers are deployment parameters.

### Point cloud

Sender: `uav/nodes/jetson_cloud_lora_odom_sender.py`

Packet framing:

```text
uint32 payload_len (network byte order)
uint32 n_points
float32 x, y, z repeated n_points times (little-endian float payload)
```

Receiver: `ground_station/nodes/dk2500_cloud_lora_odom_receiver.py`

Output topic defaults to `/cloud_from_jetson`.

### LoRa

The UAV sender reads newline-oriented LoRa serial messages such as:

```text
LORA_RX,COUNT=1,RSSI=-63,SNR=9.25,SIZE=12,MSG=HELP001
```

It parses RSSI/SNR/message metadata, attaches the current UAV pose when available, serializes a JSON object and sends one UTF-8 JSON object per line.

Ground output topic defaults to `/lora_from_jetson`.

### Odometry

The UAV sender serializes the latest pose / yaw / linear velocity as newline-delimited JSON. Ground output topic defaults to `/odom_from_jetson`.

## 2. D435 RGB transport

Sender: `uav/nodes/d435_rgb_tcp_sender.py`

Receiver: `ground_station/nodes/d435_rgb_tcp_receiver.py`

Binary frame format:

```text
4 bytes   magic = b"RGBD"
uint32    sequence
float64   ROS timestamp seconds
uint16    width
uint16    height
uint32    JPEG payload length
N bytes   JPEG payload
```

The receiver decodes JPEG and publishes `/d435/rgb_from_jetson` by default.

## 3. DK2500 -> UAV mission route / command transport

The ground planner can emit newline-delimited UTF-8 JSON packets to the Jetson mission supervisor. The mission supervisor may also receive equivalent JSON through ROS topics.

Representative command semantics include:

- route updates;
- START / RESUME;
- PAUSE / HOLD;
- STOP_TASK / MISSION_FINISHED;
- RETURN_HOME;
- TARGET_CONFIRM;
- RC_OVERRIDE.

The mission supervisor publishes route acknowledgements and execution status through ROS topics.

## Deployment note

Do not treat the historical default port numbers as a hard protocol. IPs / ports are site-specific and may change between indoor and outdoor tests. Configure both endpoints consistently.

## Experimental transport

`archive/experimental/raw_livox_to_pcl2_sender_single_socket.py` uses a different single-socket framing design (`PCL2` magic + JSON payloads). It is retained for traceability and is **not** the canonical V3 data path.
