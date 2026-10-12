// ============================================================
// COMMAND CONFIGURATION
// ============================================================

const commands = [
    { label: 'Check ROS Nodes', cmd: 'ros_nodes' },
    { label: 'System Health', cmd: 'sys_health' },
    { label: 'List Active Topics', cmd: 'ros_topics' },
    { label: 'Force Stop Movement', cmd: 'force_stop' },
    { label: 'CPU Temperature', cmd: 'cpu_temp' },
    { label: 'Camera Status', cmd: 'camera_check' },
    { label: 'Network Info', cmd: 'network_info' },
    { label: 'ROS Services', cmd: 'ros_services' },
];

// ============================================================
// DOM ELEMENTS
// ============================================================

const elements = {
    clock: document.getElementById('clock'),
    modeDot: document.getElementById('modeDot'),
    modeLabel: document.getElementById('modeLabel'),
    batteryValue: document.getElementById('batteryValue'),
    logBody: document.getElementById('logBody'),
    commandGrid: document.getElementById('commandGrid'),
    btnStart: document.getElementById('btnStart'),
    btnPause: document.getElementById('btnPause'),
    btnEmergency: document.getElementById('btnEmergency'),
    btnClearLog: document.getElementById('btnClearLog'),
    btnAddCommand: document.getElementById('btnAddCommand'),
    addCommandModal: document.getElementById('addCommandModal'),
    btnCloseModal: document.getElementById('btnCloseModal'),
    btnCancelAdd: document.getElementById('btnCancelAdd'),
    addCommandForm: document.getElementById('addCommandForm'),
    btnScreenshot: document.getElementById('btnScreenshot'),
    btnFullscreen: document.getElementById('btnFullscreen'),
};

// ============================================================
// SYSTEM LOG
// ============================================================

function addLog(message, type = 'info') {
    const logBody = elements.logBody;
    const time = new Date().toLocaleTimeString('en-US', { hour12: false });

    const entry = document.createElement('div');
    entry.className = `log-entry log-${type}`;
    entry.innerHTML = `
        <span class="log-time">[${time}]</span>
        <span class="log-msg">${escapeHtml(message)}</span>
    `;

    logBody.appendChild(entry);
    logBody.scrollTop = logBody.scrollHeight;

    // Keep only last 100 entries
    const entries = logBody.querySelectorAll('.log-entry');
    if (entries.length > 100) {
        entries[0].remove();
    }
}

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

function clearLog() {
    elements.logBody.innerHTML = '';
    addLog('Log cleared.', 'info');
}

// ============================================================
// COMMAND EXECUTION
// ============================================================

async function executeCommand(cmdName, cmdLabel = cmdName) {
    addLog(`Executing: ${cmdLabel}...`, 'info');

    try {
        const response = await fetch('/exec_command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ cmd: cmdName }),
        });

        const data = await response.json();

        if (data.success) {
            const output = data.output || 'OK (no output)';
            addLog(`[OK] ${cmdLabel}: ${output}`, 'success');
        } else {
            addLog(`[ERR] ${cmdLabel}: ${data.error}`, 'error');
        }

    } catch (error) {
        addLog(`[NET] ${cmdLabel}: ${error.message}`, 'error');
    }
}

// ============================================================
// RENDER COMMANDS
// ============================================================

function renderCommands() {
    const grid = elements.commandGrid;
    grid.innerHTML = '';

    commands.forEach(cmd => {
        const btn = document.createElement('button');
        btn.className = 'cmd-btn';
        btn.innerHTML = `
            <span class="cmd-label">${cmd.label}</span>
            <span class="cmd-id">${cmd.cmd}</span>
        `;
        btn.addEventListener('click', () => executeCommand(cmd.cmd, cmd.label));
        grid.appendChild(btn);
    });
}

function addCommandToGrid(cmd) {
    const grid = elements.commandGrid;
    const btn = document.createElement('button');
    btn.className = 'cmd-btn';
    btn.innerHTML = `
        <span class="cmd-label">${cmd.label}</span>
        <span class="cmd-id">${cmd.cmd}</span>
    `;
    btn.addEventListener('click', () => executeCommand(cmd.cmd, cmd.label));
    grid.appendChild(btn);

    commands.push(cmd);
}

// ============================================================
// ADD COMMAND MODAL
// ============================================================

function initAddCommandModal() {
    const modal = elements.addCommandModal;
    const btnAdd = elements.btnAddCommand;
    const btnClose = elements.btnCloseModal;
    const btnCancel = elements.btnCancelAdd;
    const form = elements.addCommandForm;

    function openModal() {
        modal.classList.add('active');
        document.getElementById('cmdLabel').focus();
    }

    function closeModal() {
        modal.classList.remove('active');
        form.reset();
    }

    btnAdd.addEventListener('click', openModal);
    btnClose.addEventListener('click', closeModal);
    btnCancel.addEventListener('click', closeModal);

    modal.addEventListener('click', (e) => {
        if (e.target === modal) closeModal();
    });

    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && modal.classList.contains('active')) {
            closeModal();
        }
    });

    form.addEventListener('submit', async (e) => {
        e.preventDefault();

        const label = document.getElementById('cmdLabel').value.trim();
        const cmd = document.getElementById('cmdId').value.trim();
        const command = document.getElementById('cmdCommand').value.trim();

        if (!label || !cmd || !command) {
            addLog('All fields are required.', 'error');
            return;
        }

        try {
            const response = await fetch('/api/add_command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ label, cmd, command }),
            });

            const data = await response.json();

            if (data.success) {
                addLog(`Command added: ${label} (${cmd})`, 'success');
                addCommandToGrid({ label, cmd });
                closeModal();
            } else {
                addLog(`Failed to add command: ${data.error}`, 'error');
            }

        } catch (error) {
            addLog(`Network error: ${error.message}`, 'error');
        }
    });
}

// ============================================================
// PAGE NAVIGATION
// ============================================================

function initPageNavigation() {
    const navButtons = document.querySelectorAll('.nav-btn');
    const pages = document.querySelectorAll('.page');

    navButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const targetPage = btn.getAttribute('data-page');

            // Update nav buttons
            navButtons.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            // Update pages
            pages.forEach(page => {
                page.classList.remove('active');
                if (page.id === `page-${targetPage}`) {
                    page.classList.add('active');
                }
            });
        });
    });
}

// ============================================================
// MODE BUTTONS
// ============================================================

function initModeButtons() {
    const modeButtons = document.querySelectorAll('.mode-btn');

    modeButtons.forEach(btn => {
        btn.addEventListener('click', async () => {
            const mode = btn.getAttribute('data-mode');

            // Update UI
            modeButtons.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            // Update overlay chip
            updateModeChip(mode);

            // Send to backend
            try {
                const response = await fetch('/api/mode', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ mode: mode }),
                });
                const data = await response.json();
                if (data.success) {
                    addLog(`Mode changed to: ${mode}`, 'success');
                } else {
                    addLog(`Failed to change mode: ${data.error}`, 'error');
                }
            } catch (error) {
                addLog(`Network error: ${error.message}`, 'error');
            }
        });
    });
}

function updateModeChip(mode) {
    const dot = elements.modeDot;
    const label = elements.modeLabel;

    // Remove all mode classes
    dot.classList.remove('mode-standby', 'mode-patrol', 'mode-evacuation', 'mode-manual');

    // Add current mode class
    dot.classList.add(`mode-${mode.toLowerCase()}`);
    label.textContent = mode;

    // Show/hide joystick based on mode
    if (mode === 'MANUAL') {
        joystick.show();
        addLog('Manual control enabled. Use joystick to control robot.', 'info');
    } else {
        joystick.hide();
        if (mode === 'STANDBY') {
            addLog('Robot in standby mode.', 'info');
        } else if (mode === 'PATROL') {
            addLog('Autonomous patrol mode activated.', 'success');
        } else if (mode === 'EVACUATION') {
            addLog('Emergency evacuation mode activated!', 'warning');
        }
    }
}

// ============================================================
// STATUS POLLING
// ============================================================

async function pollStatus() {
    try {
        const response = await fetch('/status');
        const data = await response.json();

        // Update battery
        elements.batteryValue.textContent = `${data.battery.toFixed(0)}%`;

        // Update mode
        updateModeChip(data.mode);

    } catch (error) {
        // Silent fail
    }
}

// ============================================================
// CLOCK
// ============================================================

function updateClock() {
    const now = new Date();
    elements.clock.textContent = now.toLocaleTimeString('en-US', {
        hour12: false,
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit',
    });
}

// ============================================================
// CAMERA CONTROLS
// ============================================================

function initCameraControls() {
    if (elements.btnScreenshot) {
        elements.btnScreenshot.addEventListener('click', () => {
            addLog('Screenshot capture - feature coming soon.', 'info');
        });
    }

    if (elements.btnFullscreen) {
        elements.btnFullscreen.addEventListener('click', () => {
            const cameraContainer = document.querySelector('.camera-fullscreen');
            if (cameraContainer) {
                if (cameraContainer.requestFullscreen) {
                    cameraContainer.requestFullscreen();
                } else if (cameraContainer.webkitRequestFullscreen) {
                    cameraContainer.webkitRequestFullscreen();
                }
                addLog('Fullscreen mode toggled.', 'info');
            }
        });
    }
}

// ============================================================
// VIRTUAL JOYSTICK
// ============================================================

const joystick = {
    canvas: null,
    ctx: null,
    active: false,
    currentX: 0,
    currentY: 0,
    centerX: 0,
    centerY: 0,
    radius: 80,
    maxLinearVel: 0.5,  // m/s
    maxAngularVel: 1.0, // rad/s
    sendInterval: null,

    init() {
        this.canvas = document.getElementById('joystickCanvas');
        if (!this.canvas) return;

        this.ctx = this.canvas.getContext('2d');
        this.centerX = this.canvas.width / 2;
        this.centerY = this.canvas.height / 2;

        // Event listeners
        this.canvas.addEventListener('mousedown', (e) => this.startDrag(e));
        this.canvas.addEventListener('mousemove', (e) => this.drag(e));
        this.canvas.addEventListener('mouseup', () => this.stopDrag());
        this.canvas.addEventListener('mouseleave', () => this.stopDrag());

        // Touch events
        this.canvas.addEventListener('touchstart', (e) => {
            e.preventDefault();
            this.startDrag(e.touches[0]);
        });
        this.canvas.addEventListener('touchmove', (e) => {
            e.preventDefault();
            this.drag(e.touches[0]);
        });
        this.canvas.addEventListener('touchend', () => this.stopDrag());

        this.draw();
    },

    startDrag(e) {
        this.active = true;
        this.updatePosition(e);
        this.startSending();
    },

    drag(e) {
        if (!this.active) return;
        this.updatePosition(e);
    },

    stopDrag() {
        this.active = false;
        this.currentX = this.centerX;
        this.currentY = this.centerY;
        this.draw();
        this.updateVelocityDisplay(0, 0);
        this.stopSending();
        this.sendVelocity(0, 0); // Stop robot
    },

    updatePosition(e) {
        const rect = this.canvas.getBoundingClientRect();
        const scaleX = this.canvas.width / rect.width;
        const scaleY = this.canvas.height / rect.height;

        let x = (e.clientX - rect.left) * scaleX;
        let y = (e.clientY - rect.top) * scaleY;

        // Calculate distance from center
        const dx = x - this.centerX;
        const dy = y - this.centerY;
        const distance = Math.sqrt(dx * dx + dy * dy);

        // Limit to radius
        if (distance > this.radius) {
            x = this.centerX + (dx / distance) * this.radius;
            y = this.centerY + (dy / distance) * this.radius;
        }

        this.currentX = x;
        this.currentY = y;

        this.draw();
        this.calculateAndSendVelocity();
    },

    draw() {
        const ctx = this.ctx;
        const width = this.canvas.width;
        const height = this.canvas.height;

        // Clear canvas
        ctx.clearRect(0, 0, width, height);

        // Draw outer circle
        ctx.beginPath();
        ctx.arc(this.centerX, this.centerY, this.radius, 0, Math.PI * 2);
        ctx.strokeStyle = '#2a2a3a';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Draw crosshair
        ctx.beginPath();
        ctx.moveTo(this.centerX - this.radius, this.centerY);
        ctx.lineTo(this.centerX + this.radius, this.centerY);
        ctx.moveTo(this.centerX, this.centerY - this.radius);
        ctx.lineTo(this.centerX, this.centerY + this.radius);
        ctx.strokeStyle = '#1e1e2a';
        ctx.lineWidth = 1;
        ctx.stroke();

        // Draw joystick handle
        ctx.beginPath();
        ctx.arc(this.currentX, this.currentY, 20, 0, Math.PI * 2);
        ctx.fillStyle = this.active ? '#3b82f6' : '#5a5a66';
        ctx.fill();
        ctx.strokeStyle = '#2563eb';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Draw direction indicator
        if (this.active) {
            ctx.beginPath();
            ctx.moveTo(this.centerX, this.centerY);
            ctx.lineTo(this.currentX, this.currentY);
            ctx.strokeStyle = 'rgba(59, 130, 246, 0.5)';
            ctx.lineWidth = 2;
            ctx.stroke();
        }
    },

    calculateAndSendVelocity() {
        // Calculate normalized position (-1 to 1)
        const dx = (this.currentX - this.centerX) / this.radius;
        const dy = (this.currentY - this.centerY) / this.radius;

        // Apply dead zone (10%)
        const deadZone = 0.1;
        const linearX = Math.abs(dy) > deadZone ? -dy : 0;  // Inverted Y axis
        const angularZ = Math.abs(dx) > deadZone ? dx : 0;

        // Scale to max velocity
        const linearVel = linearX * this.maxLinearVel;
        const angularVel = angularZ * this.maxAngularVel;

        this.updateVelocityDisplay(linearVel, angularVel);
        this.sendVelocity(linearVel, angularVel);
    },

    updateVelocityDisplay(linear, angular) {
        const linearEl = document.getElementById('linearVel');
        const angularEl = document.getElementById('angularVel');

        if (linearEl) linearEl.textContent = linear.toFixed(2);
        if (angularEl) angularEl.textContent = angular.toFixed(2);
    },

    sendVelocity(linear, angular) {
        fetch('/api/cmd_vel', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                linear_x: linear,
                angular_z: angular
            })
        }).catch(err => {
            console.error('Failed to send velocity:', err);
        });
    },

    startSending() {
        // Send velocity every 100ms while dragging
        if (this.sendInterval) return;
        this.sendInterval = setInterval(() => {
            if (this.active) {
                this.calculateAndSendVelocity();
            }
        }, 100);
    },

    stopSending() {
        if (this.sendInterval) {
            clearInterval(this.sendInterval);
            this.sendInterval = null;
        }
    },

    show() {
        const block = document.getElementById('joystickBlock');
        if (block) {
            block.style.display = 'block';
            this.init();
        }
    },

    hide() {
        const block = document.getElementById('joystickBlock');
        if (block) {
            block.style.display = 'none';
            this.stopDrag();
        }
    }
};

// ============================================================
// EVENT LISTENERS
// ============================================================

function initEventListeners() {
    if (elements.btnStart) {
        elements.btnStart.addEventListener('click', () => {
            addLog('Mission start command sent.', 'success');
        });
    }

    if (elements.btnPause) {
        elements.btnPause.addEventListener('click', () => {
            addLog('Mission pause command sent.', 'warning');
        });
    }

    if (elements.btnEmergency) {
        elements.btnEmergency.addEventListener('click', () => {
            addLog('!!! EMERGENCY STOP ACTIVATED !!!', 'error');
            executeCommand('force_stop', 'Emergency Stop');
        });
    }

    if (elements.btnClearLog) {
        elements.btnClearLog.addEventListener('click', clearLog);
    }

    const btnStopMovement = document.getElementById('btnStopMovement');
    if (btnStopMovement) {
        btnStopMovement.addEventListener('click', () => {
            joystick.stopDrag();
            addLog('Movement stopped.', 'warning');
        });
    }

    const btnResetJoystick = document.getElementById('btnResetJoystick');
    if (btnResetJoystick) {
        btnResetJoystick.addEventListener('click', () => {
            joystick.stopDrag();
            joystick.draw();
            addLog('Joystick reset.', 'info');
        });
    }
}

// ============================================================
// INITIALIZATION
// ============================================================

function init() {
    console.log('[AER Dashboard] Initializing...');

    renderCommands();
    initPageNavigation();
    initModeButtons();
    initAddCommandModal();
    initCameraControls();
    initEventListeners();

    updateClock();
    setInterval(updateClock, 1000);

    pollStatus();
    setInterval(pollStatus, 3000);

    addLog('Dashboard initialized. All systems ready.', 'success');

    console.log('[AER Dashboard] Ready.');
}

document.addEventListener('DOMContentLoaded', init);