import { useState, useEffect } from 'react';
import { 
  IconLayersIntersect, 
  IconRobot, 
  IconMapPin, 
  IconBattery2, 
  IconGauge, 
  IconRoute, 
  IconMap, 
  IconCpu, 
  IconRadar, 
  IconGyroscope, 
  IconRipple, 
  IconBroadcast, 
  IconAdjustmentsHorizontal, 
  IconSteeringWheel, 
  IconHome, 
  IconPlayerStop, 
  IconList, 
  IconChartBar, 
  IconCrosshair, 
  IconClock, 
  IconArrowRight, 
  IconUsers, 
  IconSignal 
} from '@tabler/icons-react';
import RosConnection from './components/RosConnection';

function App() {
  const [rosStatus, setRosStatus] = useState('Disconnected');
  const [telemetry, setTelemetry] = useState({
    x: 0.0,
    y: 0.0,
    battery: 100,
    speed: 0.0,
    area: 0
  });
  const [logs, setLogs] = useState([]);

  const handleRosStatusChange = (status) => {
    setRosStatus(status);
    if (status === 'Connected') {
      addLog('Sistem berhasil terhubung ke ROS Bridge', 'info');
    } else {
      addLog('Terputus dari ROS Bridge', 'err');
    }
  };

  const handleTelemetryUpdate = (data) => {
    setTelemetry(prev => ({ ...prev, ...data }));
  };

  const addLog = (msg, tag = 'info') => {
    const time = new Date().toLocaleTimeString('id-ID', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    setLogs(prev => [{ time, msg, tag }, ...prev].slice(0, 50));
  };

  const sendPrompt = (msg) => {
    addLog(`Operator: ${msg}`, 'info');
  };

  return (
    <div className="dash">
      <RosConnection 
        onStatusChange={handleRosStatusChange} 
        onTelemetryUpdate={handleTelemetryUpdate}
      />

      <div className="topbar">
        <div className="logo">
          <IconLayersIntersect className="logo-icon" />
          Autonomous Evacuation Rover — Web Dashboard
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '12px' }}>
          <span className="mode-badge"><IconRobot size={16} /> Mode: Evakuasi Aktif</span>
          <span className={`badge ${rosStatus === 'Connected' ? 'badge-ok' : 'badge-err'}`}>
            <div className={`dot ${rosStatus !== 'Connected' ? 'disconnected' : ''}`}></div>
            {rosStatus}
          </span>
        </div>
      </div>

      <div className="grid4">
        <div className="mcard">
          <div className="label"><IconMapPin size={16} /> Posisi (X, Y)</div>
          <div className="val">{telemetry.x.toFixed(1)}, {telemetry.y.toFixed(1)}</div>
          <div className="sub">meter dari origin</div>
        </div>
        <div className="mcard">
          <div className="label"><IconBattery2 size={16} /> Baterai</div>
          <div className="val" style={{ color: telemetry.battery > 20 ? 'var(--text-green)' : 'var(--text-red)' }}>
            {telemetry.battery.toFixed(0)}%
          </div>
          <div className="batt-bar">
            <div 
              className={`batt-fill ${telemetry.battery < 20 ? 'low' : telemetry.battery < 50 ? 'warn' : ''}`} 
              style={{ width: `${telemetry.battery}%` }}
            ></div>
          </div>
        </div>
        <div className="mcard">
          <div className="label"><IconGauge size={16} /> Kecepatan</div>
          <div className="val">{telemetry.speed.toFixed(2)} m/s</div>
          <div className="sub">target: 0.5 m/s</div>
        </div>
        <div className="mcard">
          <div className="label"><IconRoute size={16} /> Area terpetakan</div>
          <div className="val">{telemetry.area} m²</div>
          <div className="sub">dari 500 m² target</div>
        </div>
      </div>

      <div className="grid3">
        <div className="card">
          <div className="card-title"><IconMap size={20} /> Peta SLAM real-time</div>
          <div className="map-area">
            {/* We will replace this SVG with an actual ros2djs canvas or keep mock if no map provider exists */}
            <svg className="map-svg" viewBox="0 0 320 200" xmlns="http://www.w3.org/2000/svg">
              <rect width="320" height="200" fill="none"/>
              <rect x="20" y="20" width="280" height="160" rx="4" fill="none" stroke="var(--border-color)" strokeWidth="1.5" strokeDasharray="4 4"/>
              
              {/* Dynamic Rover Position (Mocking by moving based on telemetry) */}
              <g transform={`translate(${(telemetry.x * 10) % 200}, ${(telemetry.y * 10) % 100})`}>
                <circle cx="80" cy="140" r="5" fill="var(--accent-blue)"/>
                <circle cx="80" cy="140" r="10" fill="none" stroke="var(--accent-blue)" strokeWidth="1.5" opacity="0.4"/>
                <text x="84" y="136" fontSize="8" fill="var(--text-main)" fontWeight="600">Rover</text>
              </g>

              {/* Exit */}
              <circle cx="278" cy="95" r="6" fill="none" stroke="var(--text-green)" strokeWidth="1.5"/>
              <text x="256" y="93" fontSize="8" fill="var(--text-green)" fontWeight="600">EXIT</text>
              <circle cx="278" cy="95" r="3" fill="var(--text-green)"/>
              
              <text x="22" y="16" fontSize="8" fill="var(--text-muted)">Peta sedang dibangun...</text>
            </svg>
          </div>
          <div style={{ display: 'flex', alignItems: 'center', gap: '16px', marginTop: '16px' }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px', fontSize: '12px', color: 'var(--text-muted)', fontWeight: '500' }}>
              <div style={{ width: '12px', height: '12px', borderRadius: '2px', background: 'rgba(56, 178, 89, 0.1)', border: '1px solid rgba(56, 178, 89, 0.3)' }}></div> Area terpetakan
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px', fontSize: '12px', color: 'var(--text-muted)', fontWeight: '500' }}>
              <div style={{ width: '12px', height: '12px', borderRadius: '2px', background: 'rgba(248, 113, 113, 0.2)', border: '1px solid var(--text-red)' }}></div> Rintangan
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px', fontSize: '12px', color: 'var(--text-muted)', fontWeight: '500' }}>
              <div style={{ width: '12px', height: '12px', borderRadius: '50%', background: 'var(--accent-blue)' }}></div> Jalur A*
            </div>
          </div>
        </div>

        <div style={{ display: 'flex', flexDirection: 'column', gap: '20px' }}>
          <div className="card">
            <div className="card-title"><IconCpu size={20} /> Status sensor</div>
            <div className="sensor-row">
              <div className="srow">
                <div className="sname"><IconRadar size={16} color="var(--accent-blue)" /> LiDAR 360°</div>
                <span className={`badge ${rosStatus === 'Connected' ? 'badge-ok' : 'badge-err'}`}>{rosStatus === 'Connected' ? 'Aktif' : 'Mati'}</span>
              </div>
              <div className="srow">
                <div className="sname"><IconGyroscope size={16} color="var(--text-yellow)" /> IMU MPU-6050</div>
                <span className={`badge ${rosStatus === 'Connected' ? 'badge-ok' : 'badge-err'}`}>{rosStatus === 'Connected' ? 'Aktif' : 'Mati'}</span>
              </div>
              <div className="srow">
                <div className="sname"><IconRipple size={16} color="var(--text-green)" /> Sensor getaran</div>
                <span className="badge badge-warn">Terdeteksi</span>
              </div>
              <div className="srow">
                <div className="sname"><IconBroadcast size={16} color="var(--accent-blue)" /> WiFi / MQTT</div>
                <span className={`badge ${rosStatus === 'Connected' ? 'badge-ok' : 'badge-err'}`}>{rosStatus === 'Connected' ? 'Terhubung' : 'Terputus'}</span>
              </div>
            </div>
          </div>

          <div className="card" style={{ flex: 1 }}>
            <div className="card-title"><IconAdjustmentsHorizontal size={20} /> Kontrol HITL</div>
            <div style={{ fontSize: '12px', color: 'var(--text-muted)', marginBottom: '12px', fontWeight: '500' }}>Human in the Loop — kendali operator</div>
            <div className="hitl-row">
              <button className="hitl-btn" onClick={() => sendPrompt('Ambil alih kendali manual')}>
                <IconSteeringWheel size={16} /> Ambil alih
              </button>
              <button className="hitl-btn" onClick={() => sendPrompt('Kembali ke base')}>
                <IconHome size={16} /> Kembali ke base
              </button>
            </div>
            <div className="hitl-row">
              <button className="hitl-btn danger" onClick={() => sendPrompt('EMERGENCY STOP TRIGGERED!')}>
                <IconPlayerStop size={16} /> Emergency stop
              </button>
            </div>
          </div>
        </div>
      </div>

      <div className="sec-row">
        <div className="card">
          <div className="card-title"><IconList size={20} /> Log sistem terbaru</div>
          <div className="log-area">
            {logs.length === 0 && <div className="logrow"><span className="logmsg">Belum ada log tersedia...</span></div>}
            {logs.map((log, i) => (
              <div className="logrow" key={i}>
                <span className="logt">{log.time}</span>
                <span className="logmsg">{log.msg}</span>
                <span className={`tag t-${log.tag}`}>{log.tag === 'info' ? 'Info' : log.tag === 'warn' ? 'Warning' : log.tag === 'err' ? 'Error' : log.tag}</span>
              </div>
            ))}
          </div>
        </div>

        <div className="card">
          <div className="card-title"><IconChartBar size={20} /> Metrik performa</div>
          <div style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
            <div className="stat-pill">
              <IconCrosshair size={18} color="var(--accent-blue)" />
              <div><div className="pill-label">Akurasi SLAM (RMSE)</div></div>
              <div className="pill-val">±3.8 cm</div>
            </div>
            <div className="stat-pill">
              <IconClock size={18} color="var(--text-green)" />
              <div><div className="pill-label">Waktu replanning</div></div>
              <div className="pill-val">1.3 detik</div>
            </div>
            <div className="stat-pill">
              <IconArrowRight size={18} color="var(--text-yellow)" />
              <div><div className="pill-label">Jarak ke EXIT</div></div>
              <div className="pill-val">12.4 m</div>
            </div>
            <div className="stat-pill">
              <IconUsers size={18} color="var(--text-red)" />
              <div><div className="pill-label">Estimasi waktu evakuasi</div></div>
              <div className="pill-val">~3 menit</div>
            </div>
            <div className="stat-pill">
              <IconSignal size={18} color="var(--accent-blue)" />
              <div><div className="pill-label">Latensi WebSocket</div></div>
              <div className="pill-val">{rosStatus === 'Connected' ? '12 ms' : '-'}</div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

export default App;
