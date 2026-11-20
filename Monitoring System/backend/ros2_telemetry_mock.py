import asyncio
import json
import time
import math
import websockets

clients = set()
subscribed_topics = {}

async def handle_client(websocket):
    clients.add(websocket)
    print(f"Client connected. Total clients: {len(clients)}")
    try:
        async for message in websocket:
            data = json.loads(message)
            op = data.get('op')
            
            if op == 'subscribe':
                topic = data.get('topic')
                msg_type = data.get('type', 'unknown')
                if topic not in subscribed_topics:
                    subscribed_topics[topic] = set()
                subscribed_topics[topic].add(websocket)
                print(f"Client subscribed to {topic} [{msg_type}]")
                
            elif op == 'unsubscribe':
                topic = data.get('topic')
                if topic in subscribed_topics and websocket in subscribed_topics[topic]:
                    subscribed_topics[topic].remove(websocket)
                    print(f"Client unsubscribed from {topic}")

            elif op == 'publish':
                # Tangani perintah yang dipublish dari dashboard (HITL)
                topic = data.get('topic')
                msg = data.get('msg', {})
                handle_incoming_command(topic, msg)

            elif op == 'advertise':
                # Rosbridge protocol: acknowledge advertise
                topic = data.get('topic')
                msg_type = data.get('type', 'unknown')
                print(f"Client advertised {topic} [{msg_type}]")

            elif op == 'unadvertise':
                topic = data.get('topic')
                print(f"Client unadvertised {topic}")
    
    except websockets.exceptions.ConnectionClosed:
        print("Client connection closed.")
    finally:
        clients.discard(websocket)
        for subs in subscribed_topics.values():
            subs.discard(websocket)
        print(f"Client removed. Total clients: {len(clients)}")


def handle_incoming_command(topic, msg):
    """Tangani perintah HITL yang dikirim dari web dashboard."""
    if topic == '/rover_mode':
        mode = msg.get('data', 'unknown')
        print(f"[HITL] Mode rover diubah ke: {mode}")
    elif topic == '/rover_command':
        command = msg.get('data', 'unknown')
        print(f"[HITL] Perintah diterima: {command}")
        if command == 'emergency_stop':
            print("[HITL] *** EMERGENCY STOP TRIGGERED ***")
        elif command == 'return_to_base':
            print("[HITL] Rover diperintahkan kembali ke base.")
    elif topic == '/cmd_vel':
        lx = msg.get('linear', {}).get('x', 0)
        ly = msg.get('linear', {}).get('y', 0)
        print(f"[HITL] cmd_vel diterima: linear=({lx}, {ly})")
    else:
        print(f"[HITL] Pesan diterima di topik {topic}: {json.dumps(msg)}")


async def publish_to_subscribers(topic, msg_type, msg):
    """Kirim pesan ke semua subscriber dari topik tertentu sesuai protokol rosbridge."""
    if topic in subscribed_topics and subscribed_topics[topic]:
        payload = json.dumps({
            "op": "publish",
            "topic": topic,
            "type": msg_type,
            "msg": msg
        })
        for ws in list(subscribed_topics[topic]):
            try:
                await ws.send(payload)
            except websockets.exceptions.ConnectionClosed:
                subscribed_topics[topic].discard(ws)


async def ros_publisher():
    t = 0.0
    while True:
        if clients:
            # Publish mock odometry
            await publish_to_subscribers(
                '/odom',
                'nav_msgs/msg/Odometry',
                {
                    "pose": {
                        "pose": {
                            "position": {
                                "x": 3.0 + math.sin(t) * 2.0,
                                "y": 7.0 + math.cos(t) * 2.0,
                                "z": 0.0
                            }
                        }
                    }
                }
            )
            
            # Publish mock battery status
            await publish_to_subscribers(
                '/battery_status',
                'sensor_msgs/msg/BatteryState',
                {
                    "percentage": max(0.1, 1.0 - (t % 100) / 100.0)
                }
            )
            
            # Publish mock cmd_vel (speed)
            await publish_to_subscribers(
                '/cmd_vel',
                'geometry_msgs/msg/Twist',
                {
                    "linear": {
                        "x": 0.4 + math.sin(t * 2) * 0.1,
                        "y": 0.0,
                        "z": 0.0
                    },
                    "angular": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0
                    }
                }
            )

            # Publish mock detection results (setiap 3 detik)
            if int(t * 2) % 6 == 0:
                detections = []
                # Simulasi deteksi orang dan rintangan secara bergantian
                if int(t) % 10 < 5:
                    detections.append({
                        "label": "Orang",
                        "confidence": 0.85 + math.sin(t) * 0.1,
                        "bbox": [120, 80, 200, 280]
                    })
                if int(t) % 7 < 3:
                    detections.append({
                        "label": "Rintangan",
                        "confidence": 0.72 + math.cos(t) * 0.08,
                        "bbox": [50, 150, 100, 200]
                    })
                
                if detections:
                    await publish_to_subscribers(
                        '/detection_results',
                        'std_msgs/msg/String',
                        {
                            "data": json.dumps(detections)
                        }
                    )
        
        t += 0.5
        await asyncio.sleep(0.5)

async def main():
    print("=" * 60)
    print("  ROS 2 Telemetry Mock Server (rosbridge protocol)")
    print("  Listening on ws://localhost:9090")
    print("=" * 60)
    print()
    print("Topik yang di-publish:")
    print("  /odom            [nav_msgs/msg/Odometry]")
    print("  /battery_status  [sensor_msgs/msg/BatteryState]")
    print("  /cmd_vel         [geometry_msgs/msg/Twist]")
    print("  /detection_results [std_msgs/msg/String]")
    print()
    print("Topik yang ditangani (HITL):")
    print("  /rover_mode      [std_msgs/msg/String]")
    print("  /rover_command   [std_msgs/msg/String]")
    print("  /cmd_vel         [geometry_msgs/msg/Twist] (dari operator)")
    print()
    
    server = await websockets.serve(handle_client, "localhost", 9090)
    await asyncio.gather(server.wait_closed(), ros_publisher())

if __name__ == "__main__":
    asyncio.run(main())
