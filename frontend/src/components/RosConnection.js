import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const RosConnection = ({ onStatusChange, onTelemetryUpdate }) => {
  const [ros, setRos] = useState(null);

  useEffect(() => {
    // Inisialisasi koneksi ROSLib ke rosbridge_server
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://localhost:9090' // Default port untuk rosbridge
    });

    rosInstance.on('connection', () => {
      console.log('Connected to websocket server.');
      onStatusChange('Connected');
      subscribeToTopics(rosInstance);
    });

    rosInstance.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
      onStatusChange('Error');
    });

    rosInstance.on('close', () => {
      console.log('Connection to websocket server closed.');
      onStatusChange('Disconnected');
    });

    setRos(rosInstance);

    // Cleanup saat komponen unmount
    return () => {
      rosInstance.close();
    };
  }, []);

  const subscribeToTopics = (rosInstance) => {
    // Subscribe ke topik pose/odometry
    const poseListener = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/odom',
      messageType: 'nav_msgs/Odometry'
    });

    poseListener.subscribe((message) => {
      onTelemetryUpdate({
        x: message.pose.pose.position.x,
        y: message.pose.pose.position.y
      });
    });

    // Subscribe ke topik baterai
    const batteryListener = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/battery_status',
      messageType: 'sensor_msgs/BatteryState'
    });

    batteryListener.subscribe((message) => {
      onTelemetryUpdate({
        battery: message.percentage * 100 // assuming percentage is 0.0 to 1.0
      });
    });

    // Subscribe ke topik kecepatan (cmd_vel atau odom twist)
    const speedListener = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

    speedListener.subscribe((message) => {
      const speed = Math.sqrt(
        Math.pow(message.linear.x, 2) + Math.pow(message.linear.y, 2)
      );
      onTelemetryUpdate({ speed });
    });
  };

  // Komponen ini tidak me-render apa-apa ke UI secara langsung
  return null;
};

export default RosConnection;
