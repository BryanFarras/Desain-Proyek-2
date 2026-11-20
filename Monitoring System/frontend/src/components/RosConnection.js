import { useEffect, useRef } from 'react';
import { Ros, Topic } from 'roslib';

const RECONNECT_INITIAL_DELAY = 1000;
const RECONNECT_MAX_DELAY = 30000;
const RECONNECT_MULTIPLIER = 2;

/**
 * Komponen headless yang mengelola koneksi WebSocket ke rosbridge.
 *
 * Props:
 *  - onStatusChange(status)   : dipanggil saat status berubah ('Connected'|'Disconnected'|'Error')
 *  - onTelemetryUpdate(data)  : dipanggil tiap ada data telemetri baru
 *  - onDetectionUpdate(data)  : dipanggil tiap ada data deteksi CV baru
 *  - rosRef                   : React ref — diisi dengan ROSLIB.Ros instance agar App bisa publish
 */
const RosConnection = ({ onStatusChange, onTelemetryUpdate, onDetectionUpdate, rosRef }) => {
  // Simpan callback terbaru di ref agar useEffect tidak perlu di-recreate
  const callbacksRef = useRef({ onStatusChange, onTelemetryUpdate, onDetectionUpdate });
  callbacksRef.current = { onStatusChange, onTelemetryUpdate, onDetectionUpdate };

  useEffect(() => {
    let isActive = true;                     // guard untuk StrictMode double-mount
    let reconnectTimer = null;
    let reconnectDelay = RECONNECT_INITIAL_DELAY;
    let currentRos = null;
    const listeners = [];

    function cleanup() {
      listeners.forEach(l => { try { l.unsubscribe(); } catch (_) { /* */ } });
      listeners.length = 0;
      if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    }

    function subscribeToTopics(ros) {
      // ── /odom ──
      const odomTopic = new Topic({
        ros,
        name: '/odom',
        messageType: 'nav_msgs/msg/Odometry'
      });
      odomTopic.subscribe((msg) => {
        callbacksRef.current.onTelemetryUpdate({
          x: msg.pose.pose.position.x,
          y: msg.pose.pose.position.y
        });
      });
      listeners.push(odomTopic);

      // ── /battery_status ──
      const batteryTopic = new Topic({
        ros,
        name: '/battery_status',
        messageType: 'sensor_msgs/msg/BatteryState'
      });
      batteryTopic.subscribe((msg) => {
        callbacksRef.current.onTelemetryUpdate({
          battery: msg.percentage * 100
        });
      });
      listeners.push(batteryTopic);

      // ── /cmd_vel ──
      const velTopic = new Topic({
        ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
      });
      velTopic.subscribe((msg) => {
        const speed = Math.sqrt(
          Math.pow(msg.linear.x, 2) + Math.pow(msg.linear.y, 2)
        );
        callbacksRef.current.onTelemetryUpdate({ speed });
      });
      listeners.push(velTopic);

      // ── /detection_results ──
      const detTopic = new Topic({
        ros,
        name: '/detection_results',
        messageType: 'std_msgs/msg/String'
      });
      detTopic.subscribe((msg) => {
        try {
          const parsed = JSON.parse(msg.data);
          callbacksRef.current.onDetectionUpdate?.(parsed);
        } catch (e) {
          console.warn('Failed to parse detection results:', e);
        }
      });
      listeners.push(detTopic);
    }

    function connect() {
      if (!isActive) return;

      const url = import.meta.env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090';
      console.log(`[RosConnection] Connecting to ${url}...`);

      const ros = new Ros({ url });
      currentRos = ros;

      // Expose ke parent via ref
      if (rosRef) rosRef.current = ros;

      ros.on('connection', () => {
        if (!isActive) { ros.close(); return; }
        console.log('[RosConnection] Connected.');
        reconnectDelay = RECONNECT_INITIAL_DELAY;
        callbacksRef.current.onStatusChange('Connected');
        subscribeToTopics(ros);
      });

      ros.on('error', (err) => {
        console.error('[RosConnection] Error:', err);
        callbacksRef.current.onStatusChange('Error');
      });

      ros.on('close', () => {
        if (!isActive) return;
        console.log('[RosConnection] Closed.');
        callbacksRef.current.onStatusChange('Disconnected');
        cleanup();

        // Auto-reconnect
        console.log(`[RosConnection] Reconnecting in ${reconnectDelay / 1000}s...`);
        reconnectTimer = setTimeout(() => {
          reconnectDelay = Math.min(reconnectDelay * RECONNECT_MULTIPLIER, RECONNECT_MAX_DELAY);
          connect();
        }, reconnectDelay);
      });
    }

    connect();

    // Cleanup saat unmount
    return () => {
      isActive = false;
      cleanup();
      if (currentRos) {
        try { currentRos.close(); } catch (_) { /* */ }
      }
      if (rosRef) rosRef.current = null;
    };
  }, []); // eslint-disable-line react-hooks/exhaustive-deps
  // Empty deps — callbacks accessed via ref, so this effect runs once only.

  return null;
};

export default RosConnection;
