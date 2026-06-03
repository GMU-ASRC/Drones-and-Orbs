document.addEventListener('DOMContentLoaded', () => {

    const rosStatusText = document.getElementById('ros-status-text');
    const rosStatusPill = document.getElementById('ros-status-pill');
    const sysClock = document.getElementById('sys-clock');
    const logBody = document.getElementById('log-body');
    const btnClear = document.getElementById('btn-clear-log');
    const consoleOutput = document.getElementById('console-output');
    const consoleInput = document.getElementById('console-input');
    const consoleSend = document.getElementById('console-send');
    const flightTestStatus = document.getElementById('flight-test-status');
    const btnTestQuick = document.getElementById('btn-test-quick');
    const btnTestHover = document.getElementById('btn-test-hover');
    const btnTestMove = document.getElementById('btn-test-move');
    const btnTestAbort = document.getElementById('btn-test-abort');
    const hoverDuration = document.getElementById('hover-duration');
    const quickAltitude = document.getElementById('quick-altitude');
    const hoverAltitude = document.getElementById('hover-altitude');
    const moveAltitude = document.getElementById('move-altitude');
    const moveDistance = document.getElementById('move-distance');
    const btnTestSearch     = document.getElementById('btn-test-search');
    const btnTestFollow     = document.getElementById('btn-test-follow');
    const btnTestLand       = document.getElementById('btn-test-land');
    const btnTestFollowTimed    = document.getElementById('btn-test-follow-timed');
    const btnTestSearchHover    = document.getElementById('btn-test-search-hover');

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
        while (logBody.children.length > 200) logBody.removeChild(logBody.lastChild);
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

    function sendFlightTest(cmd) {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'flight_test', data: cmd }));
            flightTestStatus.textContent = 'Sending: ' + cmd;
            logEvent('SYS', 'Flight test command: ' + cmd);
        } else {
            flightTestStatus.textContent = 'Not connected.';
        }
    }

    btnTestQuick.addEventListener('click', () => {
        const alt = parseFloat(quickAltitude.value) || 2;
        sendFlightTest('quick_land:' + alt);
    });
    btnTestHover.addEventListener('click', () => {
        const alt = parseFloat(hoverAltitude.value) || 2;
        const sec = parseInt(hoverDuration.value) || 10;
        sendFlightTest('hover:' + sec + ':' + alt);
    });
    btnTestMove.addEventListener('click', () => {
        const alt = parseFloat(moveAltitude.value) || 2;
        const dist = parseFloat(moveDistance.value) || 5;
        sendFlightTest('move_return:' + alt + ':' + dist);
    });
    btnTestAbort.addEventListener('click', () => sendFlightTest('abort'));

    if (btnTestSearch) {
        btnTestSearch.addEventListener('click', () => {
            const pattern  = document.getElementById('search-pattern').value;
            const alt      = parseFloat(document.getElementById('search-altitude').value) || 3;
            const areaW    = parseFloat(document.getElementById('search-area-w').value) || 5;
            const areaH    = parseFloat(document.getElementById('search-area-h').value) || 5;
            const spacing  = parseFloat(document.getElementById('search-spacing').value) || 1.0;
            const targetId = parseInt(document.getElementById('search-target-id').value);
            sendFlightTest(`search:${pattern}:${alt}:${areaW}:${areaH}:${spacing}:bl:land_target:0:${targetId}`);
        });
    }

    btnTestFollow.addEventListener('click', () => {
        const targetId = parseInt(document.getElementById('follow-target-id').value) || 0;
        const alt = parseFloat(document.getElementById('follow-altitude').value) || 2;
        sendFlightTest('follow:' + targetId + ':' + alt);
    });

    btnTestLand.addEventListener('click', () => {
        sendFlightTest('land_now');
    });

    btnTestFollowTimed.addEventListener('click', () => {
        const targetId = parseInt(document.getElementById('follow-timed-target-id').value) || 0;
        const alt      = parseFloat(document.getElementById('follow-timed-altitude').value) || 2;
        const duration = parseInt(document.getElementById('follow-timed-duration').value) || 30;
        sendFlightTest('follow_timed:' + targetId + ':' + alt + ':' + duration);
    });

    btnTestSearchHover.addEventListener('click', () => {
        const pattern  = document.getElementById('search-hover-pattern').value;
        const alt      = parseFloat(document.getElementById('search-hover-altitude').value) || 3;
        const areaW    = parseFloat(document.getElementById('search-hover-area-w').value) || 5;
        const areaH    = parseFloat(document.getElementById('search-hover-area-h').value) || 5;
        const spacing  = parseFloat(document.getElementById('search-hover-spacing').value) || 1.0;
        const duration = parseFloat(document.getElementById('search-hover-duration').value) || 10;
        const roverId  = parseInt(document.getElementById('search-hover-rover-id').value) || 0;
        const targetId = parseInt(document.getElementById('search-hover-target-id').value);
        sendFlightTest(`search_hover:${pattern}:${alt}:${areaW}:${areaH}:${spacing}:bl:${duration}:${roverId}:${targetId}`);
    });

    let currentPlan = null;

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

    function el(id) { return document.getElementById(id); }

    function handleFrame(type, d) {
        switch (type) {

            case 'mavros_state': {
                el('mav-state').textContent = d.system_status || '—';
                el('mav-mode').textContent = d.mode || '—';
                const armedEl = el('mav-armed');
                armedEl.textContent = d.armed ? 'YES' : 'NO';
                armedEl.style.color = d.armed ? 'red' : 'green';
                break;
            }

            case 'mavros_rel_alt': {
                el('mav-alt-rel').textContent = d.data.toFixed(2) + ' m';
                break;
            }

            case 'mavros_heading': {
                el('mav-heading').textContent = d.data.toFixed(1) + '°';
                break;
            }

            case 'mavros_velocity': {
                el('mav-spd').textContent = d.ground_speed.toFixed(2) + ' m/s';
                break;
            }

            case 'mavros_battery': {
                const vEl = el('mav-batt-v');
                vEl.textContent = d.voltage.toFixed(2) + ' V';
                vEl.style.color = d.voltage < 14.0 && d.voltage > 1.0 ? 'red' : '';
                break;
            }

            case 'vio_odom': {
                el('pos-x').textContent = d.x.toFixed(3) + ' m';
                el('pos-y').textContent = d.y.toFixed(3) + ' m';
                el('pos-z').textContent = d.z.toFixed(3) + ' m';
                el('ori-r').textContent = d.roll.toFixed(1) + '°';
                el('ori-p').textContent = d.pitch.toFixed(1) + '°';
                el('ori-y').textContent = d.yaw.toFixed(1) + '°';
                break;
            }

            case 'vio_status': {
                const vioEl = el('vio-status');
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

            case 'flight_test_status': {
                flightTestStatus.textContent = d.text;
                logEvent('SYS', d.text);
                break;
            }

            case 'camera_frame': {
                const feed = document.getElementById('camera-feed');
                if (feed) feed.src = 'data:image/jpeg;base64,' + d.data;
                break;
            }

            default:
                break;
        }
    }

});
