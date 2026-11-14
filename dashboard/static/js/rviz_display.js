/**
 * AER Rover Web RViz2 Visualizer
 * High-performance HTML5 Canvas renderer for ROS 2 occupancy grid,
 * LiDAR scan points, Nav2 path, robot footprint, and interactive 2D Nav Goal.
 */

class RVizDisplay {
    constructor(canvasId, hudId) {
        this.canvas = document.getElementById(canvasId);
        this.hud = document.getElementById(hudId);
        if (!this.canvas) return;

        this.ctx = this.canvas.getContext('2d');

        // Viewport Transform (World <-> Canvas)
        this.scale = 40.0; // pixels per meter
        this.panX = 0.0;   // canvas pixel offset from center
        this.panY = 0.0;
        this.isDragging = false;
        this.dragStartX = 0;
        this.dragStartY = 0;

        // Interaction Tools: 'pan' | 'nav_goal'
        this.currentTool = 'pan';
        this.goalStart = null;
        this.goalCurrent = null;

        // Display Toggles
        this.layers = {
            map: true,
            laser: true,
            trail: true,
            plan: true,
            grid: true,
            footprint: true,
        };

        // Offscreen Cached Map Canvas
        this.offscreenMap = document.createElement('canvas');
        this.offscreenCtx = this.offscreenMap.getContext('2d');
        this.mapMeta = null;
        this.lastMapTimestamp = 0;

        // Robot Trail History
        this.trail = [];
        this.maxTrailPoints = 120;

        // Telemetry Data Reference
        this.telemetry = null;

        this._setupEvents();
        this._resizeCanvas();
        window.addEventListener('resize', () => this._resizeCanvas());

        // Animation Loop
        this.render = this.render.bind(this);
        requestAnimationFrame(this.render);
    }

    _resizeCanvas() {
        const container = this.canvas.parentElement;
        if (!container) return;
        const rect = container.getBoundingClientRect();
        this.canvas.width = rect.width;
        this.canvas.height = Math.max(380, rect.height);
    }

    _setupEvents() {
        const c = this.canvas;

        // Mouse Down
        c.addEventListener('mousedown', (e) => {
            const rect = c.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;

            if (this.currentTool === 'nav_goal' || this.currentTool === 'initial_pose') {
                const worldPos = this.canvasToWorld(mouseX, mouseY);
                this.goalStart = worldPos;
                this.goalCurrent = worldPos;
            } else {
                this.isDragging = true;
                this.dragStartX = mouseX - this.panX;
                this.dragStartY = mouseY - this.panY;
            }
        });

        // Mouse Move
        c.addEventListener('mousemove', (e) => {
            const rect = c.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;

            const worldPos = this.canvasToWorld(mouseX, mouseY);
            if (this.hud) {
                this.hud.innerText = `X: ${worldPos.x.toFixed(2)} m | Y: ${worldPos.y.toFixed(2)} m | Zoom: ${(this.scale).toFixed(0)} px/m`;
            }

            if ((this.currentTool === 'nav_goal' || this.currentTool === 'initial_pose') && this.goalStart) {
                this.goalCurrent = worldPos;
            } else if (this.isDragging) {
                this.panX = mouseX - this.dragStartX;
                this.panY = mouseY - this.dragStartY;
            }
        });

        // Mouse Up
        window.addEventListener('mouseup', (e) => {
            if ((this.currentTool === 'nav_goal' || this.currentTool === 'initial_pose') && this.goalStart && this.goalCurrent) {
                const dx = this.goalCurrent.x - this.goalStart.x;
                const dy = this.goalCurrent.y - this.goalStart.y;
                let yaw = Math.atan2(dy, dx);
                if (Math.hypot(dx, dy) < 0.15) {
                    yaw = (this.telemetry && this.telemetry.pose) ? this.telemetry.pose.yaw : 0.0;
                }
                if (this.currentTool === 'nav_goal') {
                    this._dispatchGoal(this.goalStart.x, this.goalStart.y, yaw);
                } else if (this.currentTool === 'initial_pose') {
                    this._dispatchInitialPose(this.goalStart.x, this.goalStart.y, yaw);
                }
                this.goalStart = null;
                this.goalCurrent = null;
                this.setTool('pan');
            }
            this.isDragging = false;
        });

        // Wheel Zoom
        c.addEventListener('wheel', (e) => {
            e.preventDefault();
            const rect = c.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;

            const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
            const newScale = Math.max(8.0, Math.min(180.0, this.scale * zoomFactor));

            // Zoom towards mouse pointer
            const worldBefore = this.canvasToWorld(mouseX, mouseY);
            this.scale = newScale;
            const worldAfter = this.canvasToWorld(mouseX, mouseY);

            this.panX += (worldAfter.x - worldBefore.x) * this.scale;
            this.panY -= (worldAfter.y - worldBefore.y) * this.scale;
        }, { passive: false });

        // Touch Support for mobile / touch kiosks
        let touchStartDist = 0;
        c.addEventListener('touchstart', (e) => {
            if (e.touches.length === 1) {
                const t = e.touches[0];
                const rect = c.getBoundingClientRect();
                this.isDragging = true;
                this.dragStartX = (t.clientX - rect.left) - this.panX;
                this.dragStartY = (t.clientY - rect.top) - this.panY;
            } else if (e.touches.length === 2) {
                touchStartDist = Math.hypot(
                    e.touches[0].clientX - e.touches[1].clientX,
                    e.touches[0].clientY - e.touches[1].clientY
                );
            }
        });

        c.addEventListener('touchmove', (e) => {
            if (e.touches.length === 1 && this.isDragging) {
                const t = e.touches[0];
                const rect = c.getBoundingClientRect();
                this.panX = (t.clientX - rect.left) - this.dragStartX;
                this.panY = (t.clientY - rect.top) - this.dragStartY;
            } else if (e.touches.length === 2) {
                const dist = Math.hypot(
                    e.touches[0].clientX - e.touches[1].clientX,
                    e.touches[0].clientY - e.touches[1].clientY
                );
                if (touchStartDist > 0) {
                    const factor = dist / touchStartDist;
                    this.scale = Math.max(8.0, Math.min(180.0, this.scale * factor));
                    touchStartDist = dist;
                }
            }
        });

        c.addEventListener('touchend', () => {
            this.isDragging = false;
            touchStartDist = 0;
        });
    }

    setTool(tool) {
        this.currentTool = tool;
        if (this.canvas) {
            this.canvas.style.cursor = (tool === 'nav_goal' || tool === 'initial_pose') ? 'crosshair' : 'grab';
        }
        const btn = document.getElementById('btnNavGoalTool');
        if (btn) {
            btn.classList.toggle('active', tool === 'nav_goal');
        }
        const btnPose = document.getElementById('btnNavInitialPoseTool');
        if (btnPose) {
            btnPose.classList.toggle('active', tool === 'initial_pose');
        }
    }

    _dispatchGoal(x, y, yaw) {
        const event = new CustomEvent('navgoal-selected', { detail: { x, y, yaw } });
        window.dispatchEvent(event);
    }

    _dispatchInitialPose(x, y, yaw) {
        const event = new CustomEvent('initialpose-selected', { detail: { x, y, yaw } });
        window.dispatchEvent(event);
    }

    // Coordinate Conversion
    worldToCanvas(wx, wy) {
        const cx = this.canvas.width / 2 + this.panX;
        const cy = this.canvas.height / 2 + this.panY;
        return {
            x: cx + wx * this.scale,
            y: cy - wy * this.scale,
        };
    }

    canvasToWorld(cx, cy) {
        const midX = this.canvas.width / 2 + this.panX;
        const midY = this.canvas.height / 2 + this.panY;
        return {
            x: (cx - midX) / this.scale,
            y: -(cy - midY) / this.scale,
        };
    }

    centerOnRobot() {
        if (!this.telemetry || !this.telemetry.pose) return;
        const rx = this.telemetry.pose.x || 0.0;
        const ry = this.telemetry.pose.y || 0.0;
        this.panX = -rx * this.scale;
        this.panY = ry * this.scale;
    }

    resetView() {
        this.scale = 40.0;
        this.panX = 0.0;
        this.panY = 0.0;
    }

    updateTelemetry(telem) {
        this.telemetry = telem;
        if (telem && telem.pose) {
            const rx = telem.pose.x;
            const ry = telem.pose.y;
            // Record breadcrumb trail
            if (this.trail.length === 0 || Math.hypot(rx - this.trail[this.trail.length - 1].x, ry - this.trail[this.trail.length - 1].y) > 0.08) {
                this.trail.push({ x: rx, y: ry });
                if (this.trail.length > this.maxTrailPoints) this.trail.shift();
            }
        }
    }

    updateMap(mapMsg) {
        if (!mapMsg || !mapMsg.received || !mapMsg.data || mapMsg.data.length === 0) return;
        if (mapMsg.timestamp && mapMsg.timestamp === this.lastMapTimestamp) return;

        this.lastMapTimestamp = mapMsg.timestamp;
        const w = mapMsg.width;
        const h = mapMsg.height;
        this.mapMeta = mapMsg;

        this.offscreenMap.width = w;
        this.offscreenMap.height = h;

        const imgData = this.offscreenCtx.createImageData(w, h);
        const d = imgData.data;
        const raw = mapMsg.data;

        // OccupancyGrid is stored row-major starting at origin bottom-left
        for (let y = 0; y < h; y++) {
            for (let x = 0; x < w; x++) {
                const srcIdx = (h - 1 - y) * w + x;
                const dstIdx = (y * w + x) * 4;
                const val = raw[srcIdx];

                if (val === -1) {
                    // Unknown space: dark transparent grid
                    d[dstIdx + 0] = 16;
                    d[dstIdx + 1] = 19;
                    d[dstIdx + 2] = 28;
                    d[dstIdx + 3] = 220;
                } else if (val === 0) {
                    // Free space: clean tactical floor
                    d[dstIdx + 0] = 225;
                    d[dstIdx + 1] = 230;
                    d[dstIdx + 2] = 238;
                    d[dstIdx + 3] = 255;
                } else if (val >= 100) {
                    // Obstacle / Wall: bright red alert
                    d[dstIdx + 0] = 239;
                    d[dstIdx + 1] = 68;
                    d[dstIdx + 2] = 68;
                    d[dstIdx + 3] = 255;
                } else {
                    // Gradient inflated space
                    d[dstIdx + 0] = 245;
                    d[dstIdx + 1] = 158;
                    d[dstIdx + 2] = 11;
                    d[dstIdx + 3] = 255;
                }
            }
        }
        this.offscreenCtx.putImageData(imgData, 0, 0);
    }

    // ----------------- RENDER PIPELINE -----------------

    render() {
        const ctx = this.ctx;
        const w = this.canvas.width;
        const h = this.canvas.height;

        ctx.clearRect(0, 0, w, h);

        // Background
        ctx.fillStyle = '#0c0e14';
        ctx.fillRect(0, 0, w, h);

        // 1. Grid & Axes Layer
        if (this.layers.grid) {
            this._drawGrid();
        }

        // 2. Occupancy Grid Map Layer
        if (this.layers.map && this.mapMeta && this.offscreenMap.width > 0) {
            this._drawOccupancyMap();
        }

        // 3. Planned Path Layer (Nav2 /plan)
        if (this.layers.plan) {
            this._drawNavPath();
        }

        // 4. Robot Trail Layer
        if (this.layers.trail) {
            this._drawTrail();
        }

        // 5. LiDAR Laser Scan Overlay
        if (this.layers.laser) {
            this._drawLidarPoints();
        }

        // 6. Robot Footprint & Heading
        if (this.layers.footprint) {
            this._drawRobot();
        }

        // 7. 2D Nav Goal Drag Arrow
        if (this.goalStart && this.goalCurrent) {
            this._drawGoalArrow();
        }

        requestAnimationFrame(this.render);
    }

    _drawGrid() {
        const ctx = this.ctx;
        const w = this.canvas.width;
        const h = this.canvas.height;

        const topLeftWorld = this.canvasToWorld(0, 0);
        const bottomRightWorld = this.canvasToWorld(w, h);

        const minX = Math.floor(topLeftWorld.x);
        const maxX = Math.ceil(bottomRightWorld.x);
        const minY = Math.floor(bottomRightWorld.y);
        const maxY = Math.ceil(topLeftWorld.y);

        ctx.lineWidth = 1;
        ctx.strokeStyle = '#1a1f2e';
        ctx.fillStyle = '#4a5568';
        ctx.font = '10px monospace';

        // Draw 1-meter vertical lines
        for (let x = minX; x <= maxX; x++) {
            const p = this.worldToCanvas(x, 0);
            ctx.beginPath();
            ctx.moveTo(p.x, 0);
            ctx.lineTo(p.x, h);
            ctx.stroke();
            if (this.scale > 20 && x % 2 === 0) {
                ctx.fillText(`${x}m`, p.x + 3, h - 8);
            }
        }

        // Draw 1-meter horizontal lines
        for (let y = minY; y <= maxY; y++) {
            const p = this.worldToCanvas(0, y);
            ctx.beginPath();
            ctx.moveTo(0, p.y);
            ctx.lineTo(w, p.y);
            ctx.stroke();
            if (this.scale > 20 && y % 2 === 0) {
                ctx.fillText(`${y}m`, 6, p.y - 3);
            }
        }

        // World Origin Axes (Red = +X East, Green = +Y North)
        const origin = this.worldToCanvas(0, 0);
        ctx.lineWidth = 2.5;

        // +X Axis (Red)
        ctx.strokeStyle = '#ef4444';
        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y);
        ctx.lineTo(origin.x + 40, origin.y);
        ctx.stroke();

        // +Y Axis (Green)
        ctx.strokeStyle = '#10b981';
        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y);
        ctx.lineTo(origin.x, origin.y - 40);
        ctx.stroke();

        // Origin marker
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(origin.x, origin.y, 3, 0, Math.PI * 2);
        ctx.fill();
    }

    _drawOccupancyMap() {
        const ctx = this.ctx;
        const res = this.mapMeta.resolution;
        const ox = this.mapMeta.origin.x;
        const oy = this.mapMeta.origin.y;
        const mapW = this.mapMeta.width * res;
        const mapH = this.mapMeta.height * res;

        // Origin in canvas coordinates
        const pTopLeft = this.worldToCanvas(ox, oy + mapH);
        const destW = mapW * this.scale;
        const destH = mapH * this.scale;

        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(this.offscreenMap, pTopLeft.x, pTopLeft.y, destW, destH);
    }

    _drawTrail() {
        if (this.trail.length < 2) return;
        const ctx = this.ctx;
        ctx.strokeStyle = 'rgba(245, 158, 11, 0.45)';
        ctx.lineWidth = 2;
        ctx.setLineDash([3, 3]);

        ctx.beginPath();
        for (let i = 0; i < this.trail.length; i++) {
            const p = this.worldToCanvas(this.trail[i].x, this.trail[i].y);
            if (i === 0) ctx.moveTo(p.x, p.y);
            else ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
    }

    _drawNavPath() {
        if (!this.telemetry || !this.telemetry.nav || !this.telemetry.nav.waypoints) return;
        const pts = this.telemetry.nav.waypoints;
        if (pts.length < 2) return;

        const ctx = this.ctx;
        ctx.strokeStyle = '#10b981';
        ctx.lineWidth = 3;
        ctx.shadowColor = '#10b981';
        ctx.shadowBlur = 8;

        ctx.beginPath();
        for (let i = 0; i < pts.length; i++) {
            const p = this.worldToCanvas(pts[i][0], pts[i][1]);
            if (i === 0) ctx.moveTo(p.x, p.y);
            else ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
        ctx.shadowBlur = 0;

        // Goal Star/Ring
        const goalPt = pts[pts.length - 1];
        const gPos = this.worldToCanvas(goalPt[0], goalPt[1]);
        ctx.fillStyle = '#10b981';
        ctx.beginPath();
        ctx.arc(gPos.x, gPos.y, 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.5;
        ctx.stroke();
    }

    _drawLidarPoints() {
        if (!this.telemetry || !this.telemetry.lidar || !this.telemetry.lidar.points) return;
        const points = this.telemetry.lidar.points;
        const rx = (this.telemetry.pose && this.telemetry.pose.x) || 0.0;
        const ry = (this.telemetry.pose && this.telemetry.pose.y) || 0.0;
        const yaw = (this.telemetry.pose && this.telemetry.pose.yaw) || 0.0;

        const cosY = Math.cos(yaw);
        const sinY = Math.sin(yaw);

        const ctx = this.ctx;
        ctx.fillStyle = '#06b6d4'; // bright cyan
        ctx.shadowColor = '#06b6d4';
        ctx.shadowBlur = 4;

        for (let i = 0; i < points.length; i++) {
            const lx = points[i][0];
            const ly = points[i][1];
            // Transform to world frame
            const wx = rx + (lx * cosY - ly * sinY);
            const wy = ry + (lx * sinY + ly * cosY);
            const p = this.worldToCanvas(wx, wy);

            ctx.fillRect(p.x - 1.5, p.y - 1.5, 3, 3);
        }
        ctx.shadowBlur = 0;
    }

    _drawRobot() {
        const telem = this.telemetry || {};
        const pose = telem.pose || { x: 0.0, y: 0.0, yaw: 0.0 };
        const p = this.worldToCanvas(pose.x, pose.y);
        const yaw = pose.yaw;

        const ctx = this.ctx;
        ctx.save();
        ctx.translate(p.x, p.y);
        ctx.rotate(-yaw); // Canvas Y is inverted relative to ROS

        // Rover dimensions (25 cm x 20 cm)
        const lengthPx = 0.25 * this.scale;
        const widthPx = 0.20 * this.scale;

        // Chassis body
        ctx.fillStyle = '#1e293b';
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 2;
        ctx.fillRect(-lengthPx / 2, -widthPx / 2, lengthPx, widthPx);
        ctx.strokeRect(-lengthPx / 2, -widthPx / 2, lengthPx, widthPx);

        // Left & Right Wheels
        ctx.fillStyle = '#0f172a';
        const wheelW = lengthPx * 0.45;
        const wheelH = widthPx * 0.25;
        ctx.fillRect(-wheelW / 2, -widthPx / 2 - wheelH / 2, wheelW, wheelH);
        ctx.fillRect(-wheelW / 2, widthPx / 2 - wheelH / 2, wheelW, wheelH);

        // Heading Front Arrow
        ctx.fillStyle = '#38bdf8';
        ctx.beginPath();
        ctx.moveTo(lengthPx / 2 + 5, 0);
        ctx.lineTo(lengthPx / 4, -widthPx / 3);
        ctx.lineTo(lengthPx / 4, widthPx / 3);
        ctx.closePath();
        ctx.fill();

        // Center LiDAR sensor
        ctx.fillStyle = '#ef4444';
        ctx.beginPath();
        ctx.arc(0, 0, Math.max(3, lengthPx * 0.15), 0, Math.PI * 2);
        ctx.fill();

        ctx.restore();

        // Coordinate tag
        ctx.fillStyle = '#e2e8f0';
        ctx.font = 'bold 11px monospace';
        ctx.fillText(
            `ROVER (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}) ${((pose.yaw * 180) / Math.PI).toFixed(0)}°`,
            p.x + 14,
            p.y - 14
        );
    }

    _drawGoalArrow() {
        const pStart = this.worldToCanvas(this.goalStart.x, this.goalStart.y);
        const pCurrent = this.worldToCanvas(this.goalCurrent.x, this.goalCurrent.y);

        const ctx = this.ctx;
        ctx.strokeStyle = '#10b981';
        ctx.fillStyle = '#10b981';
        ctx.lineWidth = 3;

        ctx.beginPath();
        ctx.arc(pStart.x, pStart.y, 6, 0, Math.PI * 2);
        ctx.fill();

        // Arrow line
        ctx.beginPath();
        ctx.moveTo(pStart.x, pStart.y);
        ctx.lineTo(pCurrent.x, pCurrent.y);
        ctx.stroke();

        // Arrow head
        const angle = Math.atan2(pCurrent.y - pStart.y, pCurrent.x - pStart.x);
        const headLen = 12;
        ctx.beginPath();
        ctx.moveTo(pCurrent.x, pCurrent.y);
        ctx.lineTo(pCurrent.x - headLen * Math.cos(angle - Math.PI / 6), pCurrent.y - headLen * Math.sin(angle - Math.PI / 6));
        ctx.lineTo(pCurrent.x - headLen * Math.cos(angle + Math.PI / 6), pCurrent.y - headLen * Math.sin(angle + Math.PI / 6));
        ctx.closePath();
        ctx.fill();
    }
}
