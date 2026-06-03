document.addEventListener('DOMContentLoaded', () => {

    const sysClock = document.getElementById('sys-clock');
    function tickClock() {
        const n = new Date();
        sysClock.textContent =
            String(n.getHours()).padStart(2, '0') + ':' +
            String(n.getMinutes()).padStart(2, '0') + ':' +
            String(n.getSeconds()).padStart(2, '0');
    }
    tickClock();
    setInterval(tickClock, 1000);

    const rosStatusText = document.getElementById('ros-status-text');
    const rosStatusPill = document.getElementById('ros-status-pill');
    const hostname = window.location.hostname || 'localhost';
    const WS_URL   = `ws://${hostname}:9090`;
    let ws = null;
    let wsRetryMs = 2000;

    function wsConnect() {
        ws = new WebSocket(WS_URL);
        ws.addEventListener('open', () => {
            rosStatusText.textContent = 'CONNECTED';
            rosStatusPill.className = 'connected';
            wsRetryMs = 2000;
        });
        ws.addEventListener('close', () => {
            rosStatusText.textContent = 'DISCONNECTED';
            rosStatusPill.className = 'error';
            setTimeout(wsConnect, wsRetryMs);
            wsRetryMs = Math.min(wsRetryMs * 1.5, 15000);
        });
        ws.addEventListener('error', () => {});
    }
    wsConnect();

    const CONV = {
        m_ft:    v => [v * 3.28084,       'ft'],
        ft_m:    v => [v / 3.28084,       'm'],
        m_km:    v => [v / 1000,          'km'],
        km_m:    v => [v * 1000,          'm'],
        m_mi:    v => [v / 1609.344,      'mi'],
        mi_m:    v => [v * 1609.344,      'm'],
        ms_kn:   v => [v * 1.94384,       'kn'],
        kn_ms:   v => [v / 1.94384,       'm/s'],
        ms_mph:  v => [v * 2.23694,       'mph'],
        mph_ms:  v => [v / 2.23694,       'm/s'],
        ms_kmh:  v => [v * 3.6,           'km/h'],
        kmh_ms:  v => [v / 3.6,           'm/s'],
        deg_rad: v => [v * Math.PI / 180, 'rad'],
        rad_deg: v => [v * 180 / Math.PI, '°'],
        c_f:     v => [v * 9/5 + 32,      '°F'],
        f_c:     v => [(v - 32) * 5/9,    '°C'],
        c_k:     v => [v + 273.15,        'K'],
        k_c:     v => [v - 273.15,        '°C'],
        pa_hpa:  v => [v / 100,           'hPa'],
        hpa_pa:  v => [v * 100,           'Pa'],
        pa_psi:  v => [v / 6894.757,      'psi'],
        psi_pa:  v => [v * 6894.757,      'Pa'],
        kg_lb:   v => [v * 2.20462,       'lb'],
        lb_kg:   v => [v / 2.20462,       'kg'],
        g_oz:    v => [v / 28.3495,       'oz'],
        oz_g:    v => [v * 28.3495,       'g'],
    };

    const convInput  = document.getElementById('conv-input');
    const convType   = document.getElementById('conv-type');
    const convResult = document.getElementById('conv-result');

    function updateConv() {
        const val = parseFloat(convInput.value);
        if (isNaN(val)) { convResult.textContent = '—'; return; }
        const fn = CONV[convType.value];
        if (!fn) { convResult.textContent = '—'; return; }
        const [result, unit] = fn(val);
        convResult.textContent = result.toFixed(5).replace(/\.?0+$/, '') + ' ' + unit;
    }

    convInput.addEventListener('input', updateConv);
    convType.addEventListener('change', updateConv);
    updateConv();

});
