document.addEventListener('DOMContentLoaded', () => {
    
    // Clock
    setInterval(() => {
        const now = new Date();
        document.getElementById('sys-clock').innerText = now.toLocaleTimeString('en-US', { hour12: false });
    }, 1000);

    const runListEl = document.getElementById('run-list');
    const logContentEl = document.getElementById('log-content');
    const videoWrapperEl = document.getElementById('video-wrapper');
    const btnRefresh = document.getElementById('btn-refresh');
    
    const dlTxt = document.getElementById('download-txt');
    const dlJsonl = document.getElementById('download-jsonl');
    const dlVideo = document.getElementById('download-video');

    function loadRuns() {
        runListEl.innerHTML = '<div class="empty-message">Loading runs...</div>';
        
        fetch('/api/logs/runs')
            .then(res => res.json())
            .then(runs => {
                if (runs.length === 0) {
                    runListEl.innerHTML = '<div class="empty-message">No mission logs found.</div>';
                    return;
                }
                
                runListEl.innerHTML = '';
                runs.forEach(run => {
                    const el = document.createElement('div');
                    el.className = 'run-item';
                    el.innerHTML = `
                        <div class="run-name">${run.name.toUpperCase().replace(/_/g, ' ')}</div>
                        <div class="run-date">${run.date} at ${run.time}</div>
                    `;
                    el.addEventListener('click', () => selectRun(run, el));
                    runListEl.appendChild(el);
                });
            })
            .catch(err => {
                runListEl.innerHTML = '<div class="empty-message" style="color:red;">Error loading runs.</div>';
                console.error(err);
            });
    }

    function selectRun(run, element) {
        // Update active class
        document.querySelectorAll('.run-item').forEach(el => el.classList.remove('active'));
        element.classList.add('active');

        // Update links
        dlTxt.href = run.txt_url;
        dlTxt.style.display = 'inline';
        dlJsonl.href = run.jsonl_url;
        dlJsonl.style.display = 'inline';
        
        if (run.video_url) {
            dlVideo.href = run.video_url;
            dlVideo.style.display = 'inline';
        } else {
            dlVideo.style.display = 'none';
        }

        // Load log text
        logContentEl.innerText = 'Loading log file...';
        fetch(run.txt_url)
            .then(res => {
                if (!res.ok) throw new Error('File not found');
                return res.text();
            })
            .then(text => {
                logContentEl.textContent = text || 'Log is empty.';
                logContentEl.scrollTop = logContentEl.scrollHeight;
            })
            .catch(err => {
                logContentEl.innerText = 'Error loading log: ' + err.message;
            });

        // Load video
        if (run.video_url) {
            videoWrapperEl.innerHTML = `
                <video controls autoplay muted>
                    <source src="${run.video_url}" type="video/mp4">
                    Your browser does not support the video tag.
                </video>
            `;
        } else {
            videoWrapperEl.innerHTML = '<div class="empty-message">No video recording available for this mission.</div>';
        }
    }

    btnRefresh.addEventListener('click', loadRuns);

    // Initial load
    loadRuns();
});