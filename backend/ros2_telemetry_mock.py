import asyncio
import json
import time
import math
import websockets

clients = set()
subscribed_topics = {}

async def handle_client(websocket):
    clients.add(websocket)
    try:
        async for message in websocket:
            data = json.loads(message)
            op = data.get('op')
            
            if op == 'subscribe':
                topic = data.get('topic')
                if topic not in subscribed_topics:
                    subscribed_topics[topic] = set()
                subscribed_topics[topic].add(websocket)
                print(f"Client subscribed to {topic}")
                
            elif op == 'unsubscribe':
                topic = data.get('topic')
                if topic in subscribed_topics and websocket in subscribed_topics[topic]:
                    subscribed_topics[topic].remove(websocket)
                    print(f"Client unsubscribed from {topic}")
    
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        clients.remove(websocket)
        for subs in subscribed_topics.values():
            if websocket in subs:
                subs.remove(websocket)

async def ros_publisher():
    t = 0.0
    while True:
        if clients:
            # Publish mock odometry
            if '/odom' in subscribed_topics and subscribed_topics['/odom']:
                odom_msg = {
                    "op": "publish",
                    "topic": "/odom",
                    "msg": {
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
                }
                for ws in subscribed_topics['/odom']:
                    try:
                        await ws.send(json.dumps(odom_msg))
                    except:
                        pass
            
            # Publish mock battery
            if '/battery_status' in subscribed_topics and subscribed_topics['/battery_status']:
                battery_msg = {
                    "op": "publish",
                    "topic": "/battery_status",
                    "msg": {
                        "percentage": max(0.1, 1.0 - (t % 100) / 100.0)
                    }
                }
                for ws in subscribed_topics['/battery_status']:
                    try:
                        await ws.send(json.dumps(battery_msg))
                    except:
                        pass
            
            # Publish mock cmd_vel (speed)
            if '/cmd_vel' in subscribed_topics and subscribed_topics['/cmd_vel']:
                speed_msg = {
                    "op": "publish",
                    "topic": "/cmd_vel",
                    "msg": {
                        "linear": {
                            "x": 0.4 + math.sin(t * 2) * 0.1,
                            "y": 0.0,
                            "z": 0.0
                        }
                    }
                }
                for ws in subscribed_topics['/cmd_vel']:
                    try:
                        await ws.send(json.dumps(speed_msg))
                    except:
                        pass
        
        t += 0.5
        await asyncio.sleep(0.5)

async def main():
    print("Starting ROS 2 Telemetry Mock Server on ws://localhost:9090...")
    server = await websockets.serve(handle_client, "localhost", 9090)
    await asyncio.gather(server.wait_closed(), ros_publisher())

if __name__ == "__main__":
    asyncio.run(main())
