/**
 * AER Rover Operations Dashboard
 * Frontend state management, HITL manual control, ROS commands, and log streaming.
 */

// Global State
let rviz = null;
let currentMode = 'STANDBY';
let currentTelemetry = null;
let maxLinearSpeed = 0.25;  // m/s
let maxAngularSpeed = 0.80; // rad/s
let isEstop = false;

// Active Keys for HITL Manual Driving
const activeKeys = {};
let driveLoopTimer = null;
let joystickVel = { linear: 0.0, angular: 0.0 };

// Initialize Dashboard
document.addEventListener('DOMContentLoaded', () => {
    // 1. Initialize RViz2 Canvas Display
    rviz = new RVizDisplay('rvizCanvas', 'rvizHud');

    // 2. Setup Tab Navigation
    setupTabs();

    // 3. Setup HITL Controls (Joystick + Keys + DPad + Sliders)
    setupHITLControls();

    // 4. Setup Command Center
    setupCommandCenter();

    // 5. Setup E-Stop & Mode Switchers
    setupModeAndEstop();

    // 6. Setup RViz Toolbar
    setupRVizToolbar();

    // 7. Start Polling Loops
    startTelemetryLoop();
    startMapLoop();
    startLogsLoop();
});

// ============================================================
// TAB NAVIGATION
// ============================================================
function setupTabs() {
    const navButtons = document.querySelectorAll('.nav-tab-btn');
    const tabPanes = document.querySelectorAll('.tab-pane');

    navButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const tabId = btn.getAttribute('data-tab');
            navButtons.forEach(b => b.classList.remove('active'));
            tabPanes.forEach(p => p.classList.remove('active'));

            btn.classList.add('active');
            const targetPane = document.getElementById(tabId);
            if (targetPane) {
                targetPane.classList.add('active');
                if (tabId === 'tab-rviz' && rviz) {
                    setTimeout(() => rviz._resizeCanvas(), 50);
                }
            }
        });
    });
}

// ============================================================
// RVIZ2 TOOLBAR & 2D NAV GOAL
// ============================================================
function setupRVizToolbar() {
    const btnCenter = document.getElementById('btnCenterRobot');
    if (btnCenter) btnCenter.addEventListener('click', () => rviz && rviz.centerOnRobot());

    const btnReset = document.getElementById('btnResetView');
    if (btnReset) btnReset.addEventListener('click', () => rviz && rviz.resetView());

    const btnGoal = document.getElementById('btnNavGoalTool');
    if (btnGoal) {
        btnGoal.addEventListener('click', () => {
            if (!rviz) return;
            const newTool = rviz.currentTool === 'nav_goal' ? 'pan' : 'nav_goal';
            rviz.setTool(newTool);
        });
    }

    // Layer Toggles
    const toggleLaser = document.getElementById('chkLayerLaser');
    if (toggleLaser) toggleLaser.addEventListener('change', (e) => rviz.layers.laser = e.target.checked);

    const toggleTrail = document.getElementById('chkLayerTrail');
    if (toggleTrail) toggleTrail.addEventListener('change', (e) => rviz.layers.trail = e.target.checked);

    const togglePlan = document.getElementById('chkLayerPlan');
    if (togglePlan) togglePlan.addEventListener('change', (e) => rviz.layers.plan = e.target.checked);

    // Goal selected event from RViz
    window.addEventListener('navgoal-selected', (e) => {
        const { x, y, yaw } = e.detail;
        fetch('/api/goal', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x, y, yaw }),
        })
        .then(r => r.json())
        .then(data => {
            if (data.success) {
                showToast(`Nav2 Goal sent: (${x.toFixed(2)}, ${y.toFixed(2)})`, 'success');
            } else {
                showToast(`Goal failed: ${data.error}`, 'error');
            }
        })
        .catch(err => showToast(`Goal error: ${err}`, 'error'));
    });
}

// ============================================================
// HITL MANUAL CONTROL (Joystick, D-Pad, Keys)
// ============================================================
function setupHITLControls() {
    // Speed sliders
    const sliderLinear = document.getElementById('sliderMaxLinear');
    const valLinear = document.getElementById('valMaxLinear');
    if (sliderLinear) {
        sliderLinear.addEventListener('input', (e) => {
            maxLinearSpeed = parseFloat(e.target.value);
            if (valLinear) valLinear.innerText = `${maxLinearSpeed.toFixed(2)} m/s`;
        });
    }

    const sliderAngular = document.getElementById('sliderMaxAngular');
    const valAngular = document.getElementById('valMaxAngular');
    if (sliderAngular) {
        sliderAngular.addEventListener('input', (e) => {
            maxAngularSpeed = parseFloat(e.target.value);
            if (valAngular) valAngular.innerText = `${maxAngularSpeed.toFixed(2)} rad/s`;
        });
    }

    // Virtual Joystick Canvas
    setupVirtualJoystick();

    // D-Pad Buttons
    const dpadMap = {
        'dpadForward':   { lin: 1.0, ang: 0.0 },
        'dpadBackward':  { lin: -1.0, ang: 0.0 },
        'dpadTurnLeft':  { lin: 0.6, ang: 0.8 },
        'dpadTurnRight': { lin: 0.6, ang: -0.8 },
        'dpadSpinLeft':  { lin: 0.0, ang: 1.0 },
        'dpadSpinRight': { lin: 0.0, ang: -1.0 },
    };

    Object.keys(dpadMap).forEach(id => {
        const btn = document.getElementById(id);
        if (!btn) return;
        const config = dpadMap[id];

        const startMove = () => {
            if (currentMode !== 'MANUAL' || isEstop) return;
            sendCmdVel(config.lin * maxLinearSpeed, config.ang * maxAngularSpeed);
        };
        const stopMove = () => {
            if (currentMode === 'MANUAL') sendCmdVel(0.0, 0.0);
        };

        btn.addEventListener('mousedown', startMove);
        btn.addEventListener('mouseup', stopMove);
        btn.addEventListener('mouseleave', stopMove);
        btn.addEventListener('touchstart', (e) => { e.preventDefault(); startMove(); });
        btn.addEventListener('touchend', (e) => { e.preventDefault(); stopMove(); });
    });

    // Coast & Brake Buttons
    const btnCoast = document.getElementById('btnCoastStop');
    if (btnCoast) btnCoast.addEventListener('click', () => sendCmdVel(0.0, 0.0));

    const btnBrake = document.getElementById('btnActiveBrake');
    if (btnBrake) btnBrake.addEventListener('click', () => sendCmdVel(0.0, 0.0));

    // Keyboard Listeners
    window.addEventListener('keydown', (e) => {
        // Ignore typing in input fields
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

        const k = e.key.toLowerCase();
        if (['w', 's', 'a', 'd', 'q', 'e', ' ', 'b', 'escape'].includes(k)) {
            e.preventDefault();
        }

        if (k === 'escape') {
            triggerEmergencyStop();
            return;
        }

        if (k === ' ' || k === 'b') {
            sendCmdVel(0.0, 0.0);
            return;
        }

        if (!activeKeys[k]) {
            activeKeys[k] = true;
            updateKeyboardDriving();
        }
    });

    window.addEventListener('keyup', (e) => {
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
        const k = e.key.toLowerCase();
        if (activeKeys[k]) {
            delete activeKeys[k];
            updateKeyboardDriving();
        }
    });
}

function updateKeyboardDriving() {
    if (currentMode !== 'MANUAL' || isEstop) return;

    let lin = 0.0;
    let ang = 0.0;

    if (activeKeys['w']) lin += 1.0;
    if (activeKeys['s']) lin -= 1.0;
    if (activeKeys['a']) ang += 0.8;
    if (activeKeys['d']) ang -= 0.8;
    if (activeKeys['q']) ang += 1.0;
    if (activeKeys['e']) ang -= 1.0;

    const targetLin = lin * maxLinearSpeed;
    const targetAng = ang * maxAngularSpeed;

    sendCmdVel(targetLin, targetAng);

    // Continuous refresh while key is held
    if (Object.keys(activeKeys).length > 0) {
        if (!driveLoopTimer) {
            driveLoopTimer = setInterval(() => {
                if (Object.keys(activeKeys).length === 0) {
                    clearInterval(driveLoopTimer);
                    driveLoopTimer = null;
                } else {
                    sendCmdVel(targetLin, targetAng);
                }
            }, 100);
        }
    } else {
        if (driveLoopTimer) {
            clearInterval(driveLoopTimer);
            driveLoopTimer = null;
        }
        sendCmdVel(0.0, 0.0);
    }
}

function setupVirtualJoystick() {
    const canvas = document.getElementById('joystickCanvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    const centerX = w / 2;
    const centerY = h / 2;
    const maxRadius = w / 2 - 18;

    let isInteracting = false;
    let stickX = centerX;
    let stickY = centerY;

    function draw() {
        ctx.clearRect(0, 0, w, h);

        // Outer Ring
        ctx.fillStyle = '#141824';
        ctx.strokeStyle = '#2d3748';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.arc(centerX, centerY, maxRadius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Crosshairs
        ctx.strokeStyle = '#232936';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(centerX, 15);
        ctx.lineTo(centerX, h - 15);
        ctx.moveTo(15, centerY);
        ctx.lineTo(w - 15, centerY);
        ctx.stroke();

        // Stick Handle
        const stickRadius = 24;
        const grad = ctx.createRadialGradient(stickX - 4, stickY - 4, 3, stickX, stickY, stickRadius);
        grad.addColorStop(0, '#38bdf8');
        grad.addColorStop(1, '#0284c7');

        ctx.fillStyle = grad;
        ctx.strokeStyle = '#e0f2fe';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(stickX, stickY, stickRadius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
    }

    draw();

    function handleMove(clientX, clientY) {
        const rect = canvas.getBoundingClientRect();
        const touchX = clientX - rect.left;
        const touchY = clientY - rect.top;

        const dx = touchX - centerX;
        const dy = touchY - centerY;
        const dist = Math.hypot(dx, dy);

        if (dist > maxRadius) {
            stickX = centerX + (dx / dist) * maxRadius;
            stickY = centerY + (dy / dist) * maxRadius;
        } else {
            stickX = touchX;
            stickY = touchY;
        }

        draw();

        // Compute normalized velocities: Forward = -dy, Turn = -dx
        const normX = (stickX - centerX) / maxRadius;
        const normY = -(stickY - centerY) / maxRadius;

        const linear = normY * maxLinearSpeed;
        const angular = -normX * maxAngularSpeed;

        joystickVel = { linear, angular };
        if (currentMode === 'MANUAL' && !isEstop) {
            sendCmdVel(linear, angular);
        }
    }

    function resetStick() {
        stickX = centerX;
        stickY = centerY;
        draw();
        joystickVel = { linear: 0.0, angular: 0.0 };
        if (currentMode === 'MANUAL') {
            sendCmdVel(0.0, 0.0);
        }
    }

    canvas.addEventListener('mousedown', (e) => {
        isInteracting = true;
        handleMove(e.clientX, e.clientY);
    });

    window.addEventListener('mousemove', (e) => {
        if (isInteracting) handleMove(e.clientX, e.clientY);
    });

    window.addEventListener('mouseup', () => {
        if (isInteracting) {
            isInteracting = false;
            resetStick();
        }
    });

    // Touch
    canvas.addEventListener('touchstart', (e) => {
        e.preventDefault();
        isInteracting = true;
        handleMove(e.touches[0].clientX, e.touches[0].clientY);
    });
    canvas.addEventListener('touchmove', (e) => {
        e.preventDefault();
        if (isInteracting) handleMove(e.touches[0].clientX, e.touches[0].clientY);
    });
    canvas.addEventListener('touchend', (e) => {
        e.preventDefault();
        isInteracting = false;
        resetStick();
    });
}

function sendCmdVel(linear, angular) {
    // Update live velocity display on UI
    const elLin = document.getElementById('dispCmdLinear');
    const elAng = document.getElementById('dispCmdAngular');
    if (elLin) elLin.innerText = `${linear.toFixed(2)} m/s`;
    if (elAng) elAng.innerText = `${angular.toFixed(2)} rad/s`;

    fetch('/api/cmd_vel', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ linear_x: linear, angular_z: angular }),
    }).catch(err => console.error('cmd_vel failed:', err));
}

// ============================================================
// MODE & EMERGENCY STOP
// ============================================================
function setupModeAndEstop() {
    // Mode Buttons
    const modeButtons = document.querySelectorAll('.mode-toggle-btn');
    modeButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const mode = btn.getAttribute('data-mode');
            setMode(mode);
        });
    });

    // Emergency Stop
    const btnEstop = document.getElementById('btnEmergencyStop');
    if (btnEstop) {
        btnEstop.addEventListener('click', triggerEmergencyStop);
    }
}

function setMode(mode) {
    fetch('/api/mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode }),
    })
    .then(r => r.json())
    .then(data => {
        if (data.success) {
            currentMode = data.mode;
            isEstop = (currentMode === 'ESTOP');
            updateModeDisplay(currentMode);
            showToast(`Mode switched to: ${currentMode}`, 'info');
        }
    })
    .catch(err => showToast(`Set mode error: ${err}`, 'error'));
}

function triggerEmergencyStop() {
    isEstop = true;
    currentMode = 'ESTOP';
    updateModeDisplay('ESTOP');

    fetch('/api/emergency_stop', { method: 'POST' })
        .then(r => r.json())
        .then(() => showToast('EMERGENCY STOP ACTIVATED!', 'error'))
        .catch(err => console.error('E-Stop error:', err));
}

function updateModeDisplay(mode) {
    const badge = document.getElementById('badgeRoverMode');
    if (badge) {
        badge.innerText = mode;
        badge.className = `status-pill pill-mode pill-${mode.toLowerCase()}`;
    }

    const modeButtons = document.querySelectorAll('.mode-toggle-btn');
    modeButtons.forEach(b => {
        b.classList.toggle('active', b.getAttribute('data-mode') === mode);
    });

    const manualOverlay = document.getElementById('manualDriveIndicator');
    if (manualOverlay) {
        manualOverlay.style.display = mode === 'MANUAL' ? 'flex' : 'none';
    }
}

// ============================================================
// ROS COMMAND CENTER
// ============================================================
function setupCommandCenter() {
    const actionButtons = document.querySelectorAll('[data-ros-cmd]');
    actionButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const cmd = btn.getAttribute('data-ros-cmd');
            runRosCommand(cmd);
        });
    });

    const btnStopProc = document.getElementById('btnStopRunningProcess');
    if (btnStopProc) {
        btnStopProc.addEventListener('click', () => {
            fetch('/api/command/stop', { method: 'POST' })
                .then(r => r.json())
                .then(d => showToast(d.message, d.success ? 'info' : 'warn'));
        });
    }

    const formCustom = document.getElementById('formCustomCommand');
    if (formCustom) {
        formCustom.addEventListener('submit', (e) => {
            e.preventDefault();
            const input = document.getElementById('inputCustomCmd');
            const val = input.value.trim();
            if (!val) return;
            fetch('/api/command/run', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command: 'custom', custom_command: val }),
            })
            .then(r => r.json())
            .then(d => {
                if (d.success) {
                    showToast(`Launched: ${val}`, 'success');
                    input.value = '';
                } else {
                    showToast(d.error, 'error');
                }
            });
        });
    }

    const btnClearLogs = document.getElementById('btnClearTerminal');
    if (btnClearLogs) {
        btnClearLogs.addEventListener('click', () => {
            fetch('/api/logs/clear', { method: 'POST' })
                .then(() => {
                    const el = document.getElementById('terminalOutput');
                    if (el) el.innerHTML = '';
                });
        });
    }
}

function runRosCommand(commandKey) {
    fetch('/api/command/run', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command: commandKey }),
    })
    .then(r => r.json())
    .then(data => {
        if (data.success) {
            showToast(`Started: ${data.started}`, 'success');
        } else {
            showToast(`Failed: ${data.error}`, 'error');
        }
    })
    .catch(err => showToast(`Error: ${err}`, 'error'));
}

// ============================================================
// POLLING LOOPS (Telemetry, Map, Logs)
// ============================================================
function startTelemetryLoop() {
    setInterval(() => {
        fetch('/api/telemetry')
            .then(r => r.json())
            .then(data => {
                currentTelemetry = data;
                updateTelemetryUI(data);
                if (rviz) rviz.updateTelemetry(data);
            })
            .catch(() => {
                const elStatus = document.getElementById('badgeRosLink');
                if (elStatus) {
                    elStatus.innerText = 'OFFLINE';
                    elStatus.className = 'status-pill pill-offline';
                }
            });
    }, 200); // 5 Hz
}

function updateTelemetryUI(t) {
    // ROS Link Pill
    const elLink = document.getElementById('badgeRosLink');
    if (elLink) {
        if (t.telemetry_streaming) {
            elLink.innerText = 'ROS LINK OK';
            elLink.className = 'status-pill pill-online';
        } else {
            elLink.innerText = 'STANDBY / IDLE';
            elLink.className = 'status-pill pill-idle';
        }
    }

    // Active Process Chip
    const elProc = document.getElementById('dispActiveProcess');
    if (elProc && t.active_process) {
        if (t.active_process.running) {
            elProc.innerHTML = `<span class="pulse-dot"></span> ${t.active_process.name} (PID: ${t.active_process.pid})`;
            elProc.className = 'active-proc-chip running';
        } else {
            elProc.innerText = 'No background ROS task active';
            elProc.className = 'active-proc-chip idle';
        }
    }

    // Mode
    if (t.mode && t.mode !== currentMode) {
        currentMode = t.mode;
        updateModeDisplay(currentMode);
    }

    // Pose
    const p = t.pose || { x: 0, y: 0, yaw: 0, yaw_deg: 0 };
    setText('valPoseX', `${p.x.toFixed(2)} m`);
    setText('valPoseY', `${p.y.toFixed(2)} m`);
    setText('valPoseYaw', `${p.yaw_deg.toFixed(1)}° (${p.yaw.toFixed(2)} rad)`);
    setText('valPoseFrame', `${p.frame} (${t.source_pose})`);

    // Motion
    const m = t.motion || { linear_vel: 0, angular_vel: 0 };
    setText('valSpeedLinear', `${m.linear_vel.toFixed(2)} m/s`);
    setText('valSpeedAngular', `${m.angular_vel.toFixed(2)} rad/s`);

    // Wheels
    const w = t.wheels || { connected: false, rpm_left: 0, rpm_right: 0, tick_left: 0, tick_right: 0 };
    setText('valWheelRpmL', `${w.rpm_left.toFixed(1)} RPM`);
    setText('valWheelRpmR', `${w.rpm_right.toFixed(1)} RPM`);
    setText('valWheelTicks', `L: ${w.tick_left} | R: ${w.tick_right}`);

    const elEsp = document.getElementById('badgeEsp32');
    if (elEsp) {
        elEsp.innerText = w.connected ? 'ESP32 LINKED' : 'ESP32 NO DATA';
        elEsp.className = `status-pill ${w.connected ? 'pill-online' : 'pill-offline'}`;
    }

    // LiDAR
    const l = t.lidar || { active: false, min_range: 0, front_distance: 0 };
    setText('valLidarMin', `${l.min_range.toFixed(2)} m`);
    setText('valLidarFront', `${l.front_distance.toFixed(2)} m`);

    const elLidar = document.getElementById('badgeLidar');
    if (elLidar) {
        elLidar.innerText = l.active ? 'LIDAR ACTIVE' : 'LIDAR OFFLINE';
        elLidar.className = `status-pill ${l.active ? 'pill-online' : 'pill-offline'}`;
    }

    // System
    const sys = t.system || { cpu_temp_c: 0, cpu_usage_pct: 0, ram_usage_pct: 0 };
    setText('valCpuTemp', `${sys.cpu_temp_c.toFixed(1)} °C`);
    setText('valCpuUsage', `${sys.cpu_usage_pct.toFixed(0)} %`);
    setText('valRamUsage', `${sys.ram_usage_pct.toFixed(0)} %`);
}

function startMapLoop() {
    setInterval(() => {
        fetch('/api/map')
            .then(r => r.json())
            .then(mapMsg => {
                if (rviz && mapMsg.received) {
                    rviz.updateMap(mapMsg);
                }
            })
            .catch(() => {});
    }, 1500); // 1.5s
}

function startLogsLoop() {
    const elTerm = document.getElementById('terminalOutput');
    const elFullTerm = document.getElementById('fullTerminalOutput');
    let lastLogCount = 0;

    setInterval(() => {
        fetch('/api/logs')
            .then(r => r.json())
            .then(logs => {
                if (logs.length !== lastLogCount) {
                    lastLogCount = logs.length;
                    const html = logs.map(l => {
                        const lvlClass = `log-${l.level.toLowerCase()}`;
                        return `<div class="term-line ${lvlClass}"><span class="term-time">[${l.time}]</span> <span class="term-lvl">[${l.level}]</span> ${escapeHtml(l.text)}</div>`;
                    }).join('');
                    if (elTerm) {
                        elTerm.innerHTML = html;
                        elTerm.scrollTop = elTerm.scrollHeight;
                    }
                    if (elFullTerm) {
                        elFullTerm.innerHTML = html;
                        elFullTerm.scrollTop = elFullTerm.scrollHeight;
                    }
                }
            })
            .catch(() => {});
    }, 1000);
}

// Helpers
function setText(id, text) {
    const el = document.getElementById(id);
    if (el) el.innerText = text;
}

function escapeHtml(str) {
    return str.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

function showToast(msg, type = 'info') {
    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    toast.innerText = msg;
    document.body.appendChild(toast);
    setTimeout(() => toast.classList.add('show'), 20);
    setTimeout(() => {
        toast.classList.remove('show');
        setTimeout(() => toast.remove(), 300);
    }, 3000);
}
