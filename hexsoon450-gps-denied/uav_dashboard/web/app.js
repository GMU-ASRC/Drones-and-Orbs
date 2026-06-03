document.addEventListener('DOMContentLoaded', () => {

    const rosStatusText = document.getElementById('ros-status-text');
    const rosStatusPill = document.getElementById('ros-status-pill');
    const sysClock = document.getElementById('sys-clock');
    const streamBadge = document.getElementById('stream-badge');
    const gpsBadge = document.getElementById('gps-fix-badge');
    const videoOverlay = document.getElementById('video-overlay');
    const cameraStream = document.getElementById('camera-stream');
    const logBody = document.getElementById('log-body');
    const btnClear = document.getElementById('btn-clear-log');
    const consoleOutput = document.getElementById('console-output');
    const consoleInput = document.getElementById('console-input');
    const consoleSend = document.getElementById('console-send');

    const flightTestStatus = document.getElementById('flight-test-status');
    const btnPreviewSearch = document.getElementById('btn-preview-search');
    const btnStartSearch = document.getElementById('btn-start-search');
    const btnTestAbort = document.getElementById('btn-test-abort');
    
    const searchPattern = document.getElementById('search-pattern');
    const searchAltitude = document.getElementById('search-altitude');
    const searchWidth = document.getElementById('search-area-w');
    const searchHeight = document.getElementById('search-area-h');
    const searchSpacing = document.getElementById('search-spacing');

    const planCanvas = document.getElementById('plan-canvas');
    const planInfo = document.getElementById('plan-info');
    const planCtx = planCanvas ? planCanvas.getContext('2d') : null;

    let hasPreviewed = false;

    function tickClock() {
        const n = new Date();
        sysClock.textContent =
            String(n.getHours()).padStart(2, '0') + ':' +
            String(n.getMinutes()).padStart(2, '0') + ':' +
            String(n.getSeconds()).padStart(2, '0');
    }
    tickClock();
    setInterval(tickClock, 1000);

    const TAG_CLASS = {
        ROS: 'tag-ros', MAV: 'tag-mav', VIO: 'tag-vio',
        SYS: 'tag-sys', ERR: 'tag-err', CAM: 'tag-vio',
    };

    function logEvent(tag, message) {
        const n = new Date();
        const ts =
            String(n.getHours()).padStart(2, '0') + ':' +
            String(n.getMinutes()).padStart(2, '0') + ':' +
            String(n.getSeconds()).padStart(2, '0') + '.' +
            String(n.getMilliseconds()).padStart(3, '0');

        const tr = document.createElement('tr');
        const tdT = document.createElement('td');
        const tdG = document.createElement('td');
        const tdM = document.createElement('td');
        tdT.textContent = ts;
        tdG.textContent = tag;
        tdG.className = TAG_CLASS[tag] || '';
        tdM.textContent = message;
        tr.appendChild(tdT);
        tr.appendChild(tdG);
        tr.appendChild(tdM);
        logBody.insertBefore(tr, logBody.firstChild);
        while (logBody.children.length > 100) logBody.removeChild(logBody.lastChild);
    }

    btnClear.addEventListener('click', () => {
        logBody.innerHTML = '';
        logEvent('SYS', 'Log cleared.');
    });

    function consolePrint(text, cls) {
        const div = document.createElement('div');
        div.textContent = text;
        if (cls) div.className = cls;
        consoleOutput.appendChild(div);
        consoleOutput.scrollTop = consoleOutput.scrollHeight;
    }

    function sendCommand(text) {
        if (!text.trim()) return;
        consolePrint('> ' + text, 'console-cmd');
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'command', data: text }));
        } else {
            consolePrint('Not connected to UAV bridge.', 'console-err');
        }
    }

    consoleSend.addEventListener('click', () => {
        sendCommand(consoleInput.value);
        consoleInput.value = '';
        consoleInput.focus();
    });

    consoleInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') {
            sendCommand(consoleInput.value);
            consoleInput.value = '';
        }
    });

    const hostname = window.location.hostname || 'localhost';

    let ws = null;
    let wsRetryMs = 2000;
    let wasConnected = false;
    const WS_URL = `ws://${hostname}:9090`;

    function wsConnect() {
        ws = new WebSocket(WS_URL);

        ws.addEventListener('open', () => {
            rosStatusText.textContent = 'CONNECTED';
            rosStatusPill.className = 'connected';
            wsRetryMs = 2000;
            if (!wasConnected) {
                logEvent('ROS', `Connected to UAV data bridge at ${WS_URL}`);
                wasConnected = true;
            }
        });

        ws.addEventListener('close', () => {
            rosStatusText.textContent = 'DISCONNECTED';
            rosStatusPill.className = 'error';
            if (wasConnected) {
                logEvent('ROS', 'Disconnected from UAV bridge.');
                wasConnected = false;
            }
            setTimeout(wsConnect, wsRetryMs);
            wsRetryMs = Math.min(wsRetryMs * 1.5, 15000);
        });

        ws.addEventListener('error', () => { });

        ws.addEventListener('message', (evt) => {
            let frame;
            try { frame = JSON.parse(evt.data); }
            catch { return; }
            handleFrame(frame.type, frame.data);
        });
    }
    wsConnect();

    let _firstFlags = {};
    function once(key, fn) {
        if (!_firstFlags[key]) { _firstFlags[key] = true; fn(); }
    }

    function handleFrame(type, d) {
        switch (type) {

            case 'mavros_state': {
                once('state', () => logEvent('MAV', 'Receiving MAVROS state.'));
                document.getElementById('mav-state').textContent = d.system_status || '—';
                document.getElementById('mav-mode').textContent = d.mode || '—';
                const armedEl = document.getElementById('mav-armed');
                const wasArmed = armedEl.dataset.armed;
                armedEl.textContent = d.armed ? 'YES' : 'NO';
                armedEl.style.color = d.armed ? 'red' : 'green';
                armedEl.dataset.armed = String(d.armed);
                if (wasArmed !== undefined && wasArmed !== String(d.armed)) {
                    logEvent('MAV', d.armed ? '⚠ Vehicle ARMED.' : 'Vehicle DISARMED.');
                }
                break;
            }

            case 'mavros_gps': {
                once('gps', () => {
                    logEvent('MAV', 'GPS fix acquired.');
                    gpsBadge.textContent = 'FIX';
                    gpsBadge.className = 'connected';
                });
                document.getElementById('mav-lat').textContent = d.latitude.toFixed(7) + '°';
                document.getElementById('mav-lon').textContent = d.longitude.toFixed(7) + '°';
                document.getElementById('mav-alt-msl').textContent = d.altitude.toFixed(2) + ' m';
                break;
            }

            case 'mavros_rel_alt': {
                const alt = d.data.toFixed(2) + ' m';
                document.getElementById('mav-alt-rel').textContent = alt;
                document.getElementById('hud-alt').textContent = 'ALT ' + alt;
                break;
            }

            case 'mavros_heading': {
                document.getElementById('mav-heading').textContent = d.data.toFixed(1) + '°';
                break;
            }

            case 'mavros_velocity': {
                const spd = d.ground_speed.toFixed(2) + ' m/s';
                document.getElementById('mav-spd').textContent = spd;
                document.getElementById('hud-spd').textContent = 'SPD ' + spd;
                break;
            }

            case 'mavros_battery': {
                once('batt', () => logEvent('MAV', 'Receiving battery telemetry.'));
                const vEl = document.getElementById('mav-batt-v');
                vEl.textContent = d.voltage.toFixed(2) + ' V';
                vEl.style.color = d.voltage < 14.0 && d.voltage > 1.0 ? 'red' : '';
                document.getElementById('mav-batt-a').textContent =
                    d.current != null ? d.current.toFixed(2) + ' A' : '— A';
                break;
            }

            case 'vio_odom': {
                once('vio', () => logEvent('VIO', 'Receiving VIO odometry stream.'));
                document.getElementById('pos-x').textContent = d.x.toFixed(3) + ' m';
                document.getElementById('pos-y').textContent = d.y.toFixed(3) + ' m';
                document.getElementById('pos-z').textContent = d.z.toFixed(3) + ' m';
                document.getElementById('ori-r').textContent = d.roll.toFixed(1) + '°';
                document.getElementById('ori-p').textContent = d.pitch.toFixed(1) + '°';
                document.getElementById('ori-y').textContent = d.yaw.toFixed(1) + '°';
                const hudAlt = document.getElementById('hud-alt');
                if (hudAlt.textContent === 'ALT — m') {
                    hudAlt.textContent = 'ALT(VIO) ' + d.z.toFixed(2) + ' m';
                }
                break;
            }

            case 'vio_status': {
                const vioEl = document.getElementById('vio-status');
                if (d.online) {
                    vioEl.textContent = 'ONLINE';
                    vioEl.style.color = 'lime';
                    vioEl.style.fontWeight = 'bold';
                } else {
                    vioEl.textContent = 'OFFLINE';
                    vioEl.style.color = 'red';
                    vioEl.style.fontWeight = 'bold';
                }
                break;
            }

            case 'imu': {
                once('imu', () => logEvent('VIO', 'Receiving IMU data stream.'));
                document.getElementById('imu-avx').textContent = d.av.x.toFixed(3);
                document.getElementById('imu-avy').textContent = d.av.y.toFixed(3);
                document.getElementById('imu-avz').textContent = d.av.z.toFixed(3);
                document.getElementById('imu-lax').textContent = d.la.x.toFixed(3);
                document.getElementById('imu-lay').textContent = d.la.y.toFixed(3);
                document.getElementById('imu-laz').textContent = d.la.z.toFixed(3);
                break;
            }

            case 'rosout': {
                logEvent(d.tag, `[${d.node}] ${d.message}`);
                break;
            }

            case 'cmd_response': {
                d.text.split('\n').forEach(line => {
                    consolePrint(line, 'console-resp');
                });
                break;
            }

            case 'camera_frame': {
                cameraStream.style.display = 'block';
                videoOverlay.style.display = 'none';
                streamBadge.textContent = 'LIVE';
                streamBadge.className = 'connected';
                cameraStream.src = 'data:image/jpeg;base64,' + d.data;
                break;
            }

            case 'active_camera': {
                const ac = document.getElementById('active-camera');
                if (ac) ac.textContent = d.text;
                break;
            }

            case 'flight_test_status': {
                if (flightTestStatus) flightTestStatus.textContent = d.text;
                break;
            }

            case 'flight_plan': {
                drawFlightPlan(d);
                break;
            }

            default:
                break;
        }
    }

    const convInput = document.getElementById('conv-input');
    const convType = document.getElementById('conv-type');
    const convResult = document.getElementById('conv-result');

    const CONV = {
        m_ft: v => [v * 3.28084, 'ft'],
        ft_m: v => [v / 3.28084, 'm'],
        ms_kn: v => [v * 1.94384, 'kn'],
        kn_ms: v => [v / 1.94384, 'm/s'],
        ms_mph: v => [v * 2.23694, 'mph'],
        mph_ms: v => [v / 2.23694, 'm/s'],
        ms_kmh: v => [v * 3.6, 'km/h'],
        kmh_ms: v => [v / 3.6, 'm/s'],
        deg_rad: v => [v * Math.PI / 180, 'rad'],
        rad_deg: v => [v * 180 / Math.PI, '°'],
        c_f: v => [v * 9 / 5 + 32, '°F'],
        f_c: v => [(v - 32) * 5 / 9, '°C'],
    };

    function updateConv() {
        if (!convInput || !convType || !convResult) return;
        const val = parseFloat(convInput.value) || 0;
        const fn = CONV[convType.value];
        if (fn) {
            const [result, unit] = fn(val);
            convResult.textContent = result.toFixed(3) + ' ' + unit;
        }
    }

    if (convInput) convInput.addEventListener('input', updateConv);
    if (convType) convType.addEventListener('change', updateConv);
    updateConv();

    function sendFlightTest(cmd) {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'flight_test', data: cmd }));
            if (flightTestStatus) flightTestStatus.textContent = 'Sending: ' + cmd;
        } else {
            if (flightTestStatus) flightTestStatus.textContent = 'Not connected.';
        }
    }

    if (btnPreviewSearch) {
        btnPreviewSearch.addEventListener('click', () => {
            const pattern = searchPattern.value;
            const alt = parseFloat(searchAltitude.value) || 2;
            const w = parseFloat(searchWidth.value) || 10;
            const h = parseFloat(searchHeight.value) || 10;
            const spacing = parseFloat(searchSpacing.value) || 1.0;
            const corner = document.getElementById('search-corner').value || 'bl';
            const roverId = parseInt(document.getElementById('search-rover-id').value) || 0;
            const targetId = parseInt(document.getElementById('search-target-id').value);

            sendFlightTest(`preview_search:${pattern}:${alt}:${w}:${h}:${spacing}:${corner}`);

            hasPreviewed = true;
            btnStartSearch.disabled = false;
            btnStartSearch.style.backgroundColor = '';
        });
    }

    if (btnStartSearch) {
        btnStartSearch.addEventListener('click', () => {
            if (!hasPreviewed) return;
            const pattern = searchPattern.value;
            const alt = parseFloat(searchAltitude.value) || 2;
            const w = parseFloat(searchWidth.value) || 10;
            const h = parseFloat(searchHeight.value) || 10;
            const spacing = parseFloat(searchSpacing.value) || 1.0;
            const corner = document.getElementById('search-corner').value || 'bl';
            const roverId = parseInt(document.getElementById('search-rover-id').value) || 0;
            const targetId = parseInt(document.getElementById('search-target-id').value);

            sendFlightTest(`search_mission:${pattern}:${alt}:${w}:${h}:${spacing}:${corner}:${roverId}:${targetId}`);
        });
    }

    if (btnTestAbort) {
        btnTestAbort.addEventListener('click', () => {
            sendFlightTest('abort');
            hasPreviewed = false;
            if (btnStartSearch) btnStartSearch.disabled = true;
        });
    }

    function drawFlightPlan(plan) {
        if (!planCanvas || !planCtx) return;
        const W = planCanvas.width;
        const H = planCanvas.height;
        const ctx = planCtx;

        ctx.clearRect(0, 0, W, H);
        ctx.fillStyle = '#1a1a1a';
        ctx.fillRect(0, 0, W, H);

        if (!plan || !plan.waypoints || plan.waypoints.length === 0) {
            if (planInfo) planInfo.textContent = 'No active plan';
            ctx.fillStyle = '#555';
            ctx.font = '14px monospace';
            ctx.textAlign = 'center';
            ctx.fillText('No active flight plan', W / 2, H / 2);
            return;
        }

        const wps = plan.waypoints;
        const activeIdx = plan.active || 0;
        const uav = plan.uav || null;
        const marker = plan.marker || null;

        if (planInfo) {
            planInfo.textContent =
                wps.length + ' waypoints — WP ' + (activeIdx + 1) + '/' + wps.length;
        }

        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;
        for (const wp of wps) {
            if (wp.x < minX) minX = wp.x;
            if (wp.x > maxX) maxX = wp.x;
            if (wp.y < minY) minY = wp.y;
            if (wp.y > maxY) maxY = wp.y;
        }
        if (uav) {
            if (uav.x < minX) minX = uav.x;
            if (uav.x > maxX) maxX = uav.x;
            if (uav.y < minY) minY = uav.y;
            if (uav.y > maxY) maxY = uav.y;
        }

        const rangeX = (maxX - minX) || 10;
        const rangeY = (maxY - minY) || 10;
        const margin = 60;
        const scaleX = (W - 2 * margin) / (rangeY * 1.2);
        const scaleY = (H - 2 * margin) / (rangeX * 1.2);
        const scale = Math.min(scaleX, scaleY);
        const cx = W / 2;
        const cy = H / 2;
        const midX = (minX + maxX) / 2;
        const midY = (minY + maxY) / 2;

        function toScreen(wx, wy) {
            return [
                cx + (wy - midY) * scale,
                cy - (wx - midX) * scale
            ];
        }

        ctx.strokeStyle = '#333';
        ctx.lineWidth = 1;
        const gridStep = Math.pow(10, Math.floor(Math.log10(rangeX * 1.2)));
        const gridStepAdj = gridStep > 1 ? gridStep : 1;
        const gridMin = Math.floor(minX / gridStepAdj) * gridStepAdj - gridStepAdj;
        const gridMax = Math.ceil(maxX / gridStepAdj) * gridStepAdj + gridStepAdj;

        for (let x = gridMin; x <= gridMax; x += gridStepAdj) {
            const p1 = toScreen(x, gridMin);
            const p2 = toScreen(x, gridMax);
            ctx.beginPath(); ctx.moveTo(p1[0], p1[1]); ctx.lineTo(p2[0], p2[1]); ctx.stroke();
            const pt = toScreen(x, 0);
            ctx.fillStyle = '#555'; ctx.font = '10px monospace';
            ctx.fillText(x.toFixed(1) + 'm', pt[0], pt[1] - 5);
        }
        for (let y = gridMin; y <= gridMax; y += gridStepAdj) {
            const p1 = toScreen(gridMin, y);
            const p2 = toScreen(gridMax, y);
            ctx.beginPath(); ctx.moveTo(p1[0], p1[1]); ctx.lineTo(p2[0], p2[1]); ctx.stroke();
            const pt = toScreen(0, y);
            ctx.fillText(y.toFixed(1) + 'm', pt[0] + 5, pt[1]);
        }

        ctx.strokeStyle = '#666';
        ctx.setLineDash([5, 5]);
        const origin = toScreen(0, 0);
        ctx.beginPath(); ctx.moveTo(origin[0], 0); ctx.lineTo(origin[0], H); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, origin[1]); ctx.lineTo(W, origin[1]); ctx.stroke();
        ctx.setLineDash([]);

        if (wps.length > 0) {
            ctx.beginPath();
            const start = toScreen(wps[0].x, wps[0].y);
            ctx.moveTo(start[0], start[1]);
            for (let i = 1; i < wps.length; i++) {
                const pt = toScreen(wps[i].x, wps[i].y);
                ctx.lineTo(pt[0], pt[1]);
            }
            ctx.strokeStyle = 'rgba(0, 165, 255, 0.4)';
            ctx.lineWidth = 3;
            ctx.stroke();

            for (let i = 0; i < wps.length; i++) {
                const pt = toScreen(wps[i].x, wps[i].y);
                ctx.beginPath();
                ctx.arc(pt[0], pt[1], 4, 0, Math.PI * 2);
                ctx.fillStyle = (i === activeIdx) ? '#00ff00' : '#00a5ff';
                ctx.fill();
                ctx.fillStyle = '#fff';
                ctx.font = '10px monospace';
                ctx.fillText(i + 1, pt[0] + 8, pt[1] - 8);
            }
        }

        if (uav) {
            const uavPt = toScreen(uav.x, uav.y);
            ctx.beginPath();
            ctx.moveTo(uavPt[0], uavPt[1] - 12);
            ctx.lineTo(uavPt[0] - 8, uavPt[1] + 8);
            ctx.lineTo(uavPt[0] + 8, uavPt[1] + 8);
            ctx.closePath();
            ctx.fillStyle = '#ff3333';
            ctx.fill();
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 1;
            ctx.stroke();
        }

        if (marker) {
            const mPt = toScreen(marker.x, marker.y);
            ctx.beginPath();
            ctx.rect(mPt[0] - 8, mPt[1] - 8, 16, 16);
            ctx.fillStyle = 'rgba(255, 0, 255, 0.5)';
            ctx.fill();
            ctx.strokeStyle = '#f0f';
            ctx.lineWidth = 2;
            ctx.stroke();
            ctx.fillStyle = '#fff';
            ctx.fillText('ID:' + marker.id, mPt[0] + 12, mPt[1] - 12);
        }
    }

});
