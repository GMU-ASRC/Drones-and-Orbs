document.addEventListener('DOMContentLoaded', () => {

    const hostname = window.location.hostname || 'localhost';
    const WS_URL = `ws://${hostname}:9090`;
    const API_URL = `http://${hostname}:8000/api/system`;

    const rosStatusText = document.getElementById('ros-status-text');
    const rosStatusPill = document.getElementById('ros-status-pill');
    const sysClock = document.getElementById('sys-clock');
    const lastUpdate = document.getElementById('last-update');

    function tickClock() {
        const n = new Date();
        sysClock.textContent =
            String(n.getHours()).padStart(2, '0') + ':' +
            String(n.getMinutes()).padStart(2, '0') + ':' +
            String(n.getSeconds()).padStart(2, '0');
    }
    tickClock();
    setInterval(tickClock, 1000);

    let ws = null;
    let wsRetryMs = 2000;
    let wasConnected = false;

    function wsConnect() {
        ws = new WebSocket(WS_URL);
        ws.addEventListener('open', () => {
            rosStatusText.textContent = 'CONNECTED';
            rosStatusPill.className = 'connected';
            wsRetryMs = 2000;
            wasConnected = true;
        });
        ws.addEventListener('close', () => {
            rosStatusText.textContent = 'DISCONNECTED';
            rosStatusPill.className = 'error';
            setTimeout(wsConnect, wsRetryMs);
            wsRetryMs = Math.min(wsRetryMs * 1.5, 15000);
        });
        ws.addEventListener('error', () => { });
    }
    wsConnect();

    function el(id) { return document.getElementById(id); }

    function setBar(barId, pct) {
        const bar = el(barId);
        const clamped = Math.min(100, Math.max(0, pct));
        bar.style.width = clamped + '%';
        bar.className = 'sys-bar';
        if (clamped > 85) bar.classList.add('sys-bar-crit');
        else if (clamped > 60) bar.classList.add('sys-bar-warn');
    }

    let prevCpuIdle = null;
    let prevCpuTotal = null;

    function applyData(d) {
        const now = new Date();
        lastUpdate.textContent = 'Updated ' +
            String(now.getHours()).padStart(2, '0') + ':' +
            String(now.getMinutes()).padStart(2, '0') + ':' +
            String(now.getSeconds()).padStart(2, '0');

        el('s-hostname').textContent = d.hostname || '—';
        el('s-kernel').textContent = d.kernel || '—';
        el('s-arch').textContent = d.arch || '—';
        el('s-uptime').textContent = d.uptime || '—';
        el('s-load').textContent = d.load || '—';

        el('s-cpu-cores').textContent = d.cpu_cores ?? '—';

        if (d._cpu_idle != null && prevCpuIdle != null) {
            const dIdle = d._cpu_idle - prevCpuIdle;
            const dTotal = d._cpu_total - prevCpuTotal;
            const pct = dTotal > 0 ? Math.round((1 - dIdle / dTotal) * 100) : 0;
            el('s-cpu-pct').textContent = pct + '%';
            setBar('bar-cpu', pct);
        } else {
            el('s-cpu-pct').textContent = '—';
        }
        prevCpuIdle = d._cpu_idle;
        prevCpuTotal = d._cpu_total;

        el('s-ram-total').textContent = d.ram_total ?? '—';
        el('s-ram-used').textContent = d.ram_used ?? '—';
        el('s-ram-pct').textContent = d.ram_pct != null ? d.ram_pct + '%' : '—';
        if (typeof d.ram_pct === 'number') setBar('bar-ram', d.ram_pct);

        el('s-disk-total').textContent = d.disk_total ?? '—';
        el('s-disk-used').textContent = d.disk_used ?? '—';
        el('s-disk-pct').textContent = d.disk_pct != null ? d.disk_pct + '%' : '—';
        if (typeof d.disk_pct === 'number') setBar('bar-disk', d.disk_pct);

        el('s-gateway').textContent = d.gateway || '—';

        const netEl = el('s-internet');
        netEl.textContent = (d.internet || '—').toUpperCase();
        netEl.style.color = d.internet === 'reachable' ? 'green' : 'red';

        const ifaceBody = el('iface-body');
        ifaceBody.innerHTML = '';
        (d.interfaces || []).forEach(iface => {
            const tr = document.createElement('tr');
            tr.innerHTML = `<td>${iface.iface}</td><td>${iface.family}</td><td>${iface.addr}</td>`;
            ifaceBody.appendChild(tr);
        });

        const nodes = d.ros_nodes || [];
        el('s-node-count').textContent = nodes.length + ' node' + (nodes.length !== 1 ? 's' : '');
        const nodeList = el('ros-nodes-list');
        if (nodes.length === 0) {
            nodeList.textContent = '— no nodes found';
        } else {
            nodeList.innerHTML = nodes.map(n => `<div>${n}</div>`).join('');
        }

        el('s-topic-count').textContent = d.ros_topics_count ?? '—';
    }

    async function fetchSystemInfo() {
        try {
            const resp = await fetch(API_URL);
            if (!resp.ok) throw new Error('HTTP ' + resp.status);
            const data = await resp.json();
            applyData(data);
        } catch (e) {
            lastUpdate.textContent = 'Fetch failed: ' + e.message;
        }
    }

    fetchSystemInfo();
    setInterval(fetchSystemInfo, 10000);

});
