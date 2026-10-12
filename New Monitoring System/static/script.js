// ============================================================
// AER DASHBOARD - UNIFIED CLIENT SCRIPT
// ============================================================

const defaultCommands = [
    { label: 'Check ROS Nodes', cmd: 'ros_nodes' },
    { label: 'System Health', cmd: 'sys_health' },
    { label: 'List Active Topics', cmd: 'ros_topics' },
    { label: 'Force Stop Movement', cmd: 'force_stop' },
    { label: 'CPU Temperature', cmd: 'cpu_temp' },
    { label: 'Camera Status', cmd: 'camera_check' },
    { label: 'Network Info', cmd: 'network_info' },
    { label: 'ROS Services', cmd: 'ros_services' },
];

let commands = [...defaultCommands];

// ============================================================
// DOM ELEMENTS REFERENCE
// ============================================================
const el = {
    clock: document.getElementById('clock'),
    modeDot: document.getElementById('modeDot'),
    modeLabel: document.getElementById('modeLabel'),
    batteryValue: document.getElementById('batteryValue'),
    hwDot: document.getElementById('hwDot'),
    hwLabel: document.getElementById('hwLabel'),
    hwChip: document.getElementById('hwChip'),
    cvDetText: document.getElementById('cvDetText'),
    headerModeBadge: document.getElementById('headerModeBadge'),
    
    // Hardware controls
    serialPortSelect: document.getElementById('serialPortSelect'),
    btnRefreshPorts: document.getElementById('btnRefreshPorts'),
    btnToggleSerial: document.getElementById('btnToggleSerial'),
    hwStatusBadge: document.getElementById('hwStatusBadge'),
    lastSerialMsg: document.getElementById('lastSerialMsg'),
    
    // Mission & control buttons
    btnStart: document.getElementById('btnStart'),
    btnPause: document.getElementById('btnPause'),
    btnReturnBase: document.getElementById('btnReturnBase'),
    btnEmergency: document.getElementById('btnEmergency'),
    btnStopMovement: document.getElementById('btnStopMovement'),
    btnBrakeMovement: document.getElementById('btnBrakeMovement'),
    btnResetJoystick: document.getElementById('btnResetJoystick'),
    
    // Telemetry displays
    telemetryCoords: document.getElementById('telemetryCoords'),
    telemetryBattery: document.getElementById('telemetryBattery'),
    telemetryBattBar: document.getElementById('telemetryBattBar'),
    telemetrySpeed: document.getElementById('telemetrySpeed'),
    telemetryArea: document.getElementById('telemetryArea'),
    slamCanvas: document.getElementById('slamCanvas'),
    cvListContainer: document.getElementById('cvListContainer'),
    pReplanning: document.getElementById('pReplanning'),
    pDistExit: document.getElementById('pDistExit'),
    pEvacEst: document.getElementById('pEvacEst'),
    
    // Sensor chips
    sLidar: document.getElementById('sLidar'),
    sImu: document.getElementById('sImu'),
    sVib: document.getElementById('sVib'),
    sWifi: document.getElementById('sWifi'),
    sSerial: document.getElementById('sSerial'),
    sCam: document.getElementById('sCam'),
    
    // Commands & logs
    commandGrid: document.getElementById('commandGrid'),
    btnAddCommand: document.getElementById('btnAddCommand'),
    addCommandModal: document.getElementById('addCommandModal'),
    btnCloseModal: document.getElementById('btnCloseModal'),
    btnCancelAdd: document.getElementById('btnCancelAdd'),
    addCommandForm: document.getElementById('addCommandForm'),
    logBody: document.getElementById('logBody'),
    btnClearLog: document.getElementById('btnClearLog'),
    
    // Camera
    btnScreenshot: document.getElementById('btnScreenshot'),
    btnFullscreen: document.getElementById('btnFullscreen')
};

// ============================================================
// SYSTEM LOGS
// ============================================================
function addLog(message, type = 'info') {
    if (!el.logBody) return;
    const time = new Date().toLocaleTimeString('en-US', { hour12: false });
    const entry = document.createElement('div');
    entry.className = `log-entry log-${type}`;
    entry.innerHTML = `
        <span class="log-time">[${time}]</span>
        <span class="log-msg">${escapeHtml(message)}</span>
    `;
    el.logBody.appendChild(entry);
    el.logBody.scrollTop = el.logBody.scrollHeight;

    const entries = el.logBody.querySelectorAll('.log-entry');
    if (entries.length > 100) entries[0].remove();
}

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

// ============================================================
// NAVIGATION
// ============================================================
function initNavigation() {
    const navButtons = document.querySelectorAll('.nav-btn');
    const pages = document.querySelectorAll('.page');

    navButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const target = btn.getAttribute('data-page');
            navButtons.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            pages.forEach(p => {
                p.classList.remove('active');
                if (p.id === `page-${target}`) p.classList.add('active');
            });

            if (target === 'telemetry') {
                drawSlamMap();
            }
        });
    });
}

// ============================================================
// OPERATING MODES
// ============================================================
function initModes() {
    const modeButtons = document.querySelectorAll('.mode-btn');

    modeButtons.forEach(btn => {
        btn.addEventListener('click', async () => {
            const mode = btn.getAttribute('data-mode');
            modeButtons.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            updateModeUI(mode);

            try {
                const res = await fetch('/api/mode', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ mode })
                });
                const data = await res.json();
                if (data.success) {
                    addLog(`Mode changed to: ${mode}`, 'success');
                } else {
                    addLog(`Failed to change mode: ${data.error}`, 'error');
                }
            } catch (err) {
                addLog(`Network error changing mode: ${err.message}`, 'error');
            }
        });
    });
}

function updateModeUI(mode) {
    const m = (mode || 'STANDBY').toUpperCase();
    if (el.modeLabel) el.modeLabel.textContent = m;
    if (el.headerModeBadge) el.headerModeBadge.textContent = m;

    if (el.modeDot) {
        el.modeDot.className = 'chip-dot';
        el.modeDot.classList.add(`mode-${m.toLowerCase()}`);
    }

    // Sync mode button state
    document.querySelectorAll('.mode-btn').forEach(b => {
        if (b.getAttribute('data-mode') === m) {
            b.classList.add('active');
        } else {
            b.classList.remove('active');
        }
    });

    const joystickBlock = document.getElementById('joystickBlock');
    if (joystickBlock) {
        if (m === 'MANUAL') {
            joystickBlock.style.opacity = '1';
        } else {
            joystickBlock.style.opacity = '0.7';
        }
    }
}

// ============================================================
// HARDWARE SERIAL CONTROLS
// ============================================================
let serialConnected = false;

async function refreshSerialPorts() {
    try {
        const res = await fetch('/api/serial/ports');
        const data = await res.json();
        
        if (el.serialPortSelect) {
            el.serialPortSelect.innerHTML = '<option value="">Auto-Detect Port</option>';
            data.ports.forEach(p => {
                const opt = document.createElement('option');
                opt.value = p.device;
                opt.textContent = `${p.device} (${p.description})`;
                if (p.device === data.current_port) opt.selected = true;
                el.serialPortSelect.appendChild(opt);
            });
        }

        updateSerialUI(data.connected, data.current_port);
    } catch (err) {
        console.error('Failed to query serial ports:', err);
    }
}

function updateSerialUI(connected, port) {
    serialConnected = connected;
    if (el.hwDot) {
        if (connected) el.hwDot.classList.add('connected');
        else el.hwDot.classList.remove('connected');
    }
    if (el.hwLabel) {
        el.hwLabel.textContent = connected ? `ESP32: ${port}` : 'ESP32: STANDBY';
    }
    if (el.hwStatusBadge) {
        el.hwStatusBadge.textContent = connected ? `Connected (${port})` : 'Disconnected';
        if (connected) el.hwStatusBadge.classList.add('connected');
        else el.hwStatusBadge.classList.remove('connected');
    }
    if (el.btnToggleSerial) {
        el.btnToggleSerial.textContent = connected ? 'Disconnect' : 'Connect';
        if (connected) {
            el.btnToggleSerial.classList.replace('btn-primary', 'btn-ghost');
        } else {
            el.btnToggleSerial.classList.replace('btn-ghost', 'btn-primary');
        }
    }
    if (el.sSerial) {
        el.sSerial.textContent = connected ? 'Terhubung' : 'Simulated';
        if (connected) el.sSerial.classList.add('ok');
        else el.sSerial.classList.remove('ok');
    }
}

async function toggleSerialConnection() {
    const selectedPort = el.serialPortSelect ? el.serialPortSelect.value : '';
    const shouldDisconnect = serialConnected;

    try {
        const res = await fetch('/api/serial/connect', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                port: selectedPort || undefined,
                disconnect: shouldDisconnect
            })
        });
        const data = await res.json();
        if (data.success) {
            updateSerialUI(data.connected, data.port);
            addLog(data.connected ? `ESP32 connected on ${data.port}` : 'ESP32 disconnected', 'success');
        } else {
            addLog(`Serial connection failed: ${data.error}`, 'error');
        }
    } catch (err) {
        addLog(`Serial connection network error: ${err.message}`, 'error');
    }
}

// ============================================================
// VIRTUAL JOYSTICK & KEYBOARD CONTROL
// ============================================================
const joystick = {
    canvas: null,
    ctx: null,
    active: false,
    currentX: 100,
    currentY: 100,
    centerX: 100,
    centerY: 100,
    radius: 70,
    maxLinearVel: 0.5,
    maxAngularVel: 1.0,
    sendInterval: null,

    init() {
        this.canvas = document.getElementById('joystickCanvas');
        if (!this.canvas) return;

        this.ctx = this.canvas.getContext('2d');
        this.centerX = this.canvas.width / 2;
        this.centerY = this.canvas.height / 2;
        this.currentX = this.centerX;
        this.currentY = this.centerY;

        // Mouse Events
        this.canvas.addEventListener('mousedown', (e) => this.startDrag(e));
        window.addEventListener('mousemove', (e) => this.drag(e));
        window.addEventListener('mouseup', () => this.stopDrag());

        // Touch Events
        this.canvas.addEventListener('touchstart', (e) => {
            e.preventDefault();
            this.startDrag(e.touches[0]);
        }, { passive: false });
        window.addEventListener('touchmove', (e) => {
            if (this.active) {
                e.preventDefault();
                this.drag(e.touches[0]);
            }
        }, { passive: false });
        window.addEventListener('touchend', () => this.stopDrag());

        this.draw();
    },

    startDrag(e) {
        this.active = true;
        this.updatePos(e);
        this.startSending();
    },

    drag(e) {
        if (!this.active) return;
        this.updatePos(e);
    },

    stopDrag() {
        if (!this.active) return;
        this.active = false;
        this.currentX = this.centerX;
        this.currentY = this.centerY;
        this.draw();
        this.updateVelocityDisplay(0, 0);
        this.stopSending();
        this.sendVelocity(0, 0);
    },

    updatePos(e) {
        const rect = this.canvas.getBoundingClientRect();
        const scaleX = this.canvas.width / rect.width;
        const scaleY = this.canvas.height / rect.height;

        let x = (e.clientX - rect.left) * scaleX;
        let y = (e.clientY - rect.top) * scaleY;

        const dx = x - this.centerX;
        const dy = y - this.centerY;
        const dist = Math.sqrt(dx * dx + dy * dy);

        if (dist > this.radius) {
            x = this.centerX + (dx / dist) * this.radius;
            y = this.centerY + (dy / dist) * this.radius;
        }

        this.currentX = x;
        this.currentY = y;
        this.draw();
        this.calcAndSend();
    },

    draw() {
        const ctx = this.ctx;
        if (!ctx) return;
        const w = this.canvas.width;
        const h = this.canvas.height;

        ctx.clearRect(0, 0, w, h);

        // Outer limit circle
        ctx.beginPath();
        ctx.arc(this.centerX, this.centerY, this.radius, 0, Math.PI * 2);
        ctx.strokeStyle = '#2a2a3a';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Crosshairs
        ctx.beginPath();
        ctx.moveTo(this.centerX - this.radius, this.centerY);
        ctx.lineTo(this.centerX + this.radius, this.centerY);
        ctx.moveTo(this.centerX, this.centerY - this.radius);
        ctx.lineTo(this.centerX, this.centerY + this.radius);
        ctx.strokeStyle = '#1e1e2a';
        ctx.lineWidth = 1;
        ctx.stroke();

        // Line to handle
        if (this.active) {
            ctx.beginPath();
            ctx.moveTo(this.centerX, this.centerY);
            ctx.lineTo(this.currentX, this.currentY);
            ctx.strokeStyle = 'rgba(59, 130, 246, 0.4)';
            ctx.lineWidth = 2;
            ctx.stroke();
        }

        // Joystick Handle Knob
        ctx.beginPath();
        ctx.arc(this.currentX, this.currentY, 18, 0, Math.PI * 2);
        ctx.fillStyle = this.active ? '#3b82f6' : '#4b4b5a';
        ctx.fill();
        ctx.strokeStyle = '#2563eb';
        ctx.lineWidth = 2;
        ctx.stroke();
    },

    calcAndSend() {
        const dx = (this.currentX - this.centerX) / this.radius;
        const dy = (this.currentY - this.centerY) / this.radius;

        const deadzone = 0.08;
        const linearX = Math.abs(dy) > deadzone ? -dy : 0;
        const angularZ = Math.abs(dx) > deadzone ? dx : 0;

        const linVel = linearX * this.maxLinearVel;
        const angVel = angularZ * this.maxAngularVel;

        this.updateVelocityDisplay(linVel, angVel);
        this.sendVelocity(linVel, angVel);
    },

    updateVelocityDisplay(lin, ang) {
        const linEl = document.getElementById('linearVel');
        const angEl = document.getElementById('angularVel');
        if (linEl) linEl.textContent = lin.toFixed(2);
        if (angEl) angEl.textContent = ang.toFixed(2);
    },

    sendVelocity(lin, ang) {
        fetch('/api/cmd_vel', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ linear_x: lin, angular_z: ang })
        }).catch(() => {});
    },

    startSending() {
        if (this.sendInterval) return;
        this.sendInterval = setInterval(() => {
            if (this.active) this.calcAndSend();
        }, 120);
    },

    stopSending() {
        if (this.sendInterval) {
            clearInterval(this.sendInterval);
            this.sendInterval = null;
        }
    }
};

// Keyboard listener mapping directly to ESP32 / cmd_vel
function initKeyboardControl() {
    window.addEventListener('keydown', (e) => {
        // Prevent key capture inside inputs
        if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;

        const k = e.key.toLowerCase();
        let handled = true;

        if (k === 'w') {
            joystick.sendVelocity(0.4, 0.0);
            joystick.updateVelocityDisplay(0.4, 0.0);
        } else if (k === 's') {
            joystick.sendVelocity(-0.35, 0.0);
            joystick.updateVelocityDisplay(-0.35, 0.0);
        } else if (k === 'a') {
            joystick.sendVelocity(0.2, -0.6);
            joystick.updateVelocityDisplay(0.2, -0.6);
        } else if (k === 'd') {
            joystick.sendVelocity(0.2, 0.6);
            joystick.updateVelocityDisplay(0.2, 0.6);
        } else if (k === 'q') {
            joystick.sendVelocity(0.0, -0.8);
            joystick.updateVelocityDisplay(0.0, -0.8);
        } else if (k === 'e') {
            joystick.sendVelocity(0.0, 0.8);
            joystick.updateVelocityDisplay(0.0, 0.8);
        } else if (k === ' ') {
            fetch('/api/serial/send', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cmd: ' ' })
            });
            joystick.sendVelocity(0, 0);
            joystick.updateVelocityDisplay(0, 0);
        } else if (k === 'b') {
            fetch('/api/serial/send', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cmd: 'b' })
            });
            joystick.sendVelocity(0, 0);
            joystick.updateVelocityDisplay(0, 0);
        } else {
            handled = false;
        }

        if (handled && e.key === ' ') e.preventDefault();
    });

    window.addEventListener('keyup', (e) => {
        if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
        const k = e.key.toLowerCase();
        if (['w', 's', 'a', 'd', 'q', 'e'].includes(k)) {
            joystick.sendVelocity(0, 0);
            joystick.updateVelocityDisplay(0, 0);
        }
    });
}

// ============================================================
// REAL-TIME SLAM MAP CANVAS
// ============================================================
let currentRoverState = { x: 2.4, y: 4.8, yaw: 0.78 };
let pathHistory = [];

function drawSlamMap() {
    const canvas = el.slamCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    // Arena background & grid
    ctx.fillStyle = '#0a0a0f';
    ctx.fillRect(0, 0, w, h);

    // Coordinate grid (every 40px)
    ctx.strokeStyle = '#181824';
    ctx.lineWidth = 1;
    for (let x = 0; x < w; x += 30) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, h);
        ctx.stroke();
    }
    for (let y = 0; y < h; y += 30) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
    }

    // Origin Base Station
    const ox = 50, oy = h - 50;
    ctx.fillStyle = '#3a3a4a';
    ctx.beginPath();
    ctx.arc(ox, oy, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#6a6a7a';
    ctx.font = '9px monospace';
    ctx.fillText('BASE (0,0)', ox - 18, oy + 16);

    // Exit Point
    const ex = w - 60, ey = 60;
    ctx.strokeStyle = '#10b981';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(ex, ey, 9, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = '#10b981';
    ctx.beginPath();
    ctx.arc(ex, ey, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.font = 'bold 9px monospace';
    ctx.fillText('EXIT', ex - 10, ey - 14);

    // Map scaling: 1 meter = 24 pixels
    const scale = 24;
    const rx = ox + currentRoverState.x * scale;
    const ry = oy - currentRoverState.y * scale;

    // Track path history
    if (pathHistory.length === 0 || Math.hypot(rx - pathHistory[pathHistory.length - 1].x, ry - pathHistory[pathHistory.length - 1].y) > 4) {
        pathHistory.push({ x: rx, y: ry });
        if (pathHistory.length > 80) pathHistory.shift();
    }

    // Draw trail
    if (pathHistory.length > 1) {
        ctx.strokeStyle = 'rgba(234, 179, 8, 0.4)';
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(pathHistory[0].x, pathHistory[0].y);
        for (let i = 1; i < pathHistory.length; i++) {
            ctx.lineTo(pathHistory[i].x, pathHistory[i].y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
    }

    // Static / Detected Obstacles
    const obstacles = [
        { x: ox + 4.2 * scale, y: oy - 3.8 * scale, r: 12 },
        { x: ox + 7.5 * scale, y: oy - 6.5 * scale, r: 16 },
        { x: ox + 2.0 * scale, y: oy - 7.0 * scale, r: 10 }
    ];

    obstacles.forEach((obs, idx) => {
        ctx.fillStyle = 'rgba(239, 68, 68, 0.25)';
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(obs.x, obs.y, obs.r, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = '#fca5a5';
        ctx.font = '8px monospace';
        ctx.fillText(`OBS-${idx + 1}`, obs.x - 12, obs.y + 3);
    });

    // Draw Rover Body (Circle + Heading pointer)
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-currentRoverState.yaw);

    // Rover aura
    ctx.fillStyle = 'rgba(59, 130, 246, 0.2)';
    ctx.beginPath();
    ctx.arc(0, 0, 14, 0, Math.PI * 2);
    ctx.fill();

    // Rover circle
    ctx.fillStyle = '#3b82f6';
    ctx.beginPath();
    ctx.arc(0, 0, 7, 0, Math.PI * 2);
    ctx.fill();

    // Direction arrow
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(14, 0);
    ctx.stroke();

    ctx.restore();

    // Rover Label
    ctx.fillStyle = '#e4e4e8';
    ctx.font = 'bold 9px monospace';
    ctx.fillText(`ROVER (${currentRoverState.x.toFixed(1)}, ${currentRoverState.y.toFixed(1)})`, rx + 12, ry - 6);
}

// ============================================================
// STATUS POLLING & TELEMETRY UPDATES
// ============================================================
async function pollTelemetry() {
    try {
        const res = await fetch('/status');
        const data = await res.json();

        // Battery
        const batt = data.battery || 0;
        if (el.batteryValue) el.batteryValue.textContent = `${batt.toFixed(0)}%`;
        if (el.telemetryBattery) el.telemetryBattery.textContent = `${batt.toFixed(0)}% (${data.battery_voltage || 12.4}V)`;
        if (el.telemetryBattBar) {
            el.telemetryBattBar.style.width = `${batt}%`;
            el.telemetryBattBar.style.background = batt > 40 ? 'var(--accent-green)' : (batt > 20 ? 'var(--accent-orange)' : 'var(--accent-red)');
        }

        // Mode
        updateModeUI(data.mode);

        // Hardware Serial status
        updateSerialUI(data.hardware_serial_connected, data.serial_port);

        // Telemetry coordinates & speed
        currentRoverState.x = data.x || 0;
        currentRoverState.y = data.y || 0;
        currentRoverState.yaw = data.yaw || 0;

        if (el.telemetryCoords) el.telemetryCoords.textContent = `${(data.x || 0).toFixed(1)}, ${(data.y || 0).toFixed(1)}`;
        if (el.telemetrySpeed) el.telemetrySpeed.textContent = `${(data.speed || 0).toFixed(2)} m/s`;
        if (el.telemetryArea) el.telemetryArea.textContent = `${(data.area || 0).toFixed(1)} m²`;

        // SLAM Metrics
        if (el.pReplanning) el.pReplanning.textContent = `${data.replanning_time || 1.2} detik`;
        if (el.pDistExit) el.pDistExit.textContent = `${data.dist_to_exit || 10.2} m`;
        if (el.pEvacEst) el.pEvacEst.textContent = `~${data.evacuation_est_min || 2.8} menit`;

        // Sensors
        if (el.sLidar) el.sLidar.textContent = data.lidar_ok ? 'Aktif' : 'Off';
        if (el.sImu) el.sImu.textContent = data.imu_ok ? 'Aktif' : 'Off';
        if (el.sVib) el.sVib.textContent = data.vibration_ok ? 'Normal' : 'Getaran!';

        // CV Obstacle Detections
        const dets = data.detections || [];
        if (dets.length > 0) {
            const first = dets[0];
            if (el.cvDetText) el.cvDetText.textContent = `${first.label} ${(first.confidence * 100).toFixed(0)}% [${first.sector}]`;
        } else {
            if (el.cvDetText) el.cvDetText.textContent = 'Jalur Bebas Rintangan';
        }

        if (el.cvListContainer) {
            el.cvListContainer.innerHTML = '';
            if (dets.length === 0) {
                el.cvListContainer.innerHTML = '<div style="color:var(--text-muted);font-size:11px;padding:4px;">Tidak ada rintangan di zona pandang kamera.</div>';
            } else {
                dets.forEach(d => {
                    const row = document.createElement('div');
                    row.className = 'cv-item';
                    row.innerHTML = `
                        <span class="cv-tag">${escapeHtml(d.label)}</span>
                        <span class="cv-conf">${(d.confidence * 100).toFixed(0)}% confidence</span>
                        <span class="cv-sector">${escapeHtml(d.sector)}</span>
                    `;
                    el.cvListContainer.appendChild(row);
                });
            }
        }

        // Draw SLAM Map
        drawSlamMap();

    } catch (err) {
        // Silent network retry
    }
}

// Clock updater
function updateClock() {
    if (!el.clock) return;
    const now = new Date();
    el.clock.textContent = now.toLocaleTimeString('en-US', { hour12: false });
}

// ============================================================
// COMMANDS TAB EXECUTION
// ============================================================
async function executeCommand(cmdName, cmdLabel = cmdName) {
    addLog(`Running command: ${cmdLabel}...`, 'info');

    try {
        const res = await fetch('/exec_command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ cmd: cmdName })
        });
        const data = await res.json();

        if (data.success) {
            addLog(`[OK] ${cmdLabel}:\n${data.output}`, 'success');
        } else {
            addLog(`[ERR] ${cmdLabel}: ${data.error || 'Execution failed'}`, 'error');
        }
    } catch (err) {
        addLog(`[NET ERR] ${cmdLabel}: ${err.message}`, 'error');
    }
}

function renderCommands() {
    if (!el.commandGrid) return;
    el.commandGrid.innerHTML = '';

    commands.forEach(c => {
        const btn = document.createElement('button');
        btn.className = 'cmd-btn';
        btn.innerHTML = `
            <span class="cmd-label">${c.label}</span>
            <span class="cmd-id">${c.cmd}</span>
        `;
        btn.addEventListener('click', () => executeCommand(c.cmd, c.label));
        el.commandGrid.appendChild(btn);
    });
}

function initCommandModal() {
    if (!el.btnAddCommand || !el.addCommandModal) return;

    el.btnAddCommand.addEventListener('click', () => {
        el.addCommandModal.classList.add('active');
        document.getElementById('cmdLabel').focus();
    });

    const close = () => {
        el.addCommandModal.classList.remove('active');
        if (el.addCommandForm) el.addCommandForm.reset();
    };

    if (el.btnCloseModal) el.btnCloseModal.addEventListener('click', close);
    if (el.btnCancelAdd) el.btnCancelAdd.addEventListener('click', close);

    if (el.addCommandForm) {
        el.addCommandForm.addEventListener('submit', (e) => {
            e.preventDefault();
            const label = document.getElementById('cmdLabel').value.trim();
            const cmd = document.getElementById('cmdId').value.trim();
            if (label && cmd) {
                commands.push({ label, cmd });
                renderCommands();
                addLog(`Custom command added: ${label} [${cmd}]`, 'success');
                close();
            }
        });
    }
}

// ============================================================
// HITL BUTTONS
// ============================================================
function initHitlActions() {
    if (el.btnStart) {
        el.btnStart.addEventListener('click', () => {
            fetch('/api/mission', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command: 'start' })
            });
            addLog('Mission Started (Autonomous Patrol)', 'success');
        });
    }

    if (el.btnPause) {
        el.btnPause.addEventListener('click', () => {
            fetch('/api/mission', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command: 'pause' })
            });
            addLog('Mission Paused', 'warning');
        });
    }

    if (el.btnReturnBase) {
        el.btnReturnBase.addEventListener('click', () => {
            fetch('/api/return_to_base', { method: 'POST' });
            addLog('HITL Operator: Return to Base command sent', 'info');
        });
    }

    if (el.btnEmergency) {
        el.btnEmergency.addEventListener('click', () => {
            fetch('/api/emergency_stop', { method: 'POST' });
            addLog('🚨 EMERGENCY STOP ACTIVATED: Rover stopped immediately!', 'error');
        });
    }

    if (el.btnStopMovement) {
        el.btnStopMovement.addEventListener('click', () => {
            fetch('/api/serial/send', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cmd: ' ' })
            });
            joystick.sendVelocity(0, 0);
            addLog('Coast stop applied', 'info');
        });
    }

    if (el.btnBrakeMovement) {
        el.btnBrakeMovement.addEventListener('click', () => {
            fetch('/api/serial/send', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cmd: 'b' })
            });
            joystick.sendVelocity(0, 0);
            addLog('Electric brake applied', 'warning');
        });
    }

    if (el.btnResetJoystick) {
        el.btnResetJoystick.addEventListener('click', () => {
            joystick.stopDrag();
            addLog('Joystick center reset', 'info');
        });
    }

    if (el.btnClearLog) {
        el.btnClearLog.addEventListener('click', () => {
            if (el.logBody) el.logBody.innerHTML = '';
            addLog('Log cleared', 'info');
        });
    }

    if (el.btnRefreshPorts) {
        el.btnRefreshPorts.addEventListener('click', refreshSerialPorts);
    }

    if (el.btnToggleSerial) {
        el.btnToggleSerial.addEventListener('click', toggleSerialConnection);
    }

    // Camera actions
    if (el.btnScreenshot) {
        el.btnScreenshot.addEventListener('click', () => {
            window.open('/api/snapshot', '_blank');
            addLog('Snapshot captured', 'info');
        });
    }

    if (el.btnFullscreen) {
        el.btnFullscreen.addEventListener('click', () => {
            const camContainer = document.querySelector('.camera-fullscreen');
            if (camContainer) {
                if (!document.fullscreenElement) {
                    camContainer.requestFullscreen?.() || camContainer.webkitRequestFullscreen?.();
                } else {
                    document.exitFullscreen?.();
                }
            }
        });
    }
}

// ============================================================
// SYSTEM INIT
// ============================================================
function init() {
    console.log('[AER Unified Dashboard] Initializing...');

    initNavigation();
    initModes();
    joystick.init();
    initKeyboardControl();
    initHitlActions();
    renderCommands();
    initCommandModal();

    refreshSerialPorts();

    updateClock();
    setInterval(updateClock, 1000);

    pollTelemetry();
    setInterval(pollTelemetry, 1500);

    addLog('System online. AER Monitoring and Hardware Bridge initialized.', 'success');
}

document.addEventListener('DOMContentLoaded', init);