// Firebase REST API 클라이언트 (모듈 없음)
// Firebase Realtime Database REST 방식으로 접근

const FIREBASE_CONFIG = {
    databaseURL: "https://rokey-b-3-default-rtdb.firebaseio.com",
    apiKey: "AIzaSyCVaEaIp1lyqLlvKR7rBFDpLNyp3Iavx48"
};

// Chart 인스턴스
let forceChart = null;
let chartData = { labels: [], data: [] };

// Firebase 폴링 타이머
let pollInterval = null;
let lastTimestamp = null;

function startFirebaseListener() {
    console.log("📡 Starting Firebase real-time listener (polling mode)...");
    
    // 기존 폴링 종료
    if (pollInterval) {
        clearInterval(pollInterval);
    }
    
    // 500ms마다 데이터 폴링
    pollInterval = setInterval(() => {
        pollFirebaseData();
    }, 500);
    
    // 즉시 한 번 실행
    pollFirebaseData();
}

function pollFirebaseData() {
    const url = `${FIREBASE_CONFIG.databaseURL}/one.json?auth=${FIREBASE_CONFIG.apiKey}`;
    
    fetch(url)
        .then(response => {
            if (response.status === 401) {
                console.error("❌ Firebase 401 Unauthorized - checking security rules...");
                updateConnectionStatus(false);
                return null;
            }
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            return response.json();
        })
        .then(data => {
            if (data === null) return;
            
            // 타임스탬프로 새로운 데이터인지 확인
            if (data && data.timestamp !== lastTimestamp) {
                lastTimestamp = data.timestamp;
                console.log("📥 Firebase data received (polling):", data);
                updateDashboard(data);
                updateConnectionStatus(true);
            }
        })
        .catch(error => {
            console.error("❌ Polling error:", error);
            updateConnectionStatus(false);
        });
}

function updateDashboard(data) {
    if (!data) return;
    
    // 연결 상태 업데이트
    updateConnectionStatus(true);
    
    // 상태 박스
    if (data.status) {
        const statusEl = document.getElementById('status-box');
        statusEl.textContent = data.status;
        statusEl.className = data.status;
    }
    
    // 운영 시간 (operational_time 또는 operationalTime)
    const opTime = data.operational_time || data.operationalTime || 0;
    if (opTime !== undefined) {
        document.getElementById('operational-time').textContent = opTime.toFixed(2);
    }
    
    // 로봇 위치 (pos 또는 position)
    const pos = data.pos || data.position;
    if (pos) {
        document.getElementById('pos-x').textContent = (pos.x || 0).toFixed(2);
        document.getElementById('pos-y').textContent = (pos.y || 0).toFixed(2);
        document.getElementById('pos-z').textContent = (pos.z || 0).toFixed(2);
    }
    
    // 힘 데이터 (force_z 또는 force.z)
    const forceZ = data.force_z || (data.force?.z) || 0;
    document.getElementById('force-z').textContent = forceZ.toFixed(2);
    
    // 차트에 데이터 추가
    addChartData(forceZ);
    
    // 시스템 정보
    if (data.system_info || data.systemInfo) {
        const sysInfo = data.system_info || data.systemInfo;
        document.getElementById('system-name').textContent = sysInfo.name || 'N/A';
        document.getElementById('system-version').textContent = sysInfo.version || 'N/A';
        document.getElementById('system-device').textContent = sysInfo.device_id || sysInfo.deviceId || 'N/A';
    }
    
    // 관절 데이터 (J1-J6 각도)
    if (data.joints && Array.isArray(data.joints)) {
        const jointNames = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
        for (let i = 0; i < 6; i++) {
            const angle = data.joints[i] || 0;
            // 게이지 값 업데이트
            const gaugeFill = document.getElementById(`joint-gauge-${i}`);
            const gaugeValue = document.getElementById(`joint-value-${i}`);
            if (gaugeFill) {
                // -180 ~ 180 범위를 0 ~ 100%로 정규화
                const percentage = ((angle + 180) / 360) * 100;
                gaugeFill.style.width = percentage + '%';
                // 색상: 중앙(0도)=청색, 극단값(±180도)=빨강
                const hue = 240 - (Math.abs(angle) / 180 * 180);
                gaugeFill.style.backgroundColor = `hsl(${hue}, 100%, 50%)`;
            }
            if (gaugeValue) {
                gaugeValue.textContent = angle.toFixed(1);
            }
        }
    }
    
    // 로봇 로그 메시지
    if (data.logs && Array.isArray(data.logs)) {
        updateRobotLogs(data.logs);
    }
    
    // 데이터 그리드
    if (data.timestamp) {
        document.getElementById('grid-status').textContent = data.status || '-';
        document.getElementById('grid-force').textContent = forceZ.toFixed(2) + ' N';
        document.getElementById('grid-pos-x').textContent = (pos?.x || 0).toFixed(2) + ' mm';
        document.getElementById('grid-pos-y').textContent = (pos?.y || 0).toFixed(2) + ' mm';
        document.getElementById('grid-pos-z').textContent = (pos?.z || 0).toFixed(2) + ' mm';
        document.getElementById('grid-optime').textContent = opTime.toFixed(2) + ' h';
        
        // timestamp는 unix timestamp (초 단위)
        const date = new Date(data.timestamp * 1000);
        document.getElementById('grid-timestamp').textContent = date.toLocaleString('ko-KR');
    }
}

function updateRobotLogs(logs) {
    const logConsole = document.getElementById('log-console');
    if (!logConsole) return;
    
    // 로그 콘솔에 메시지 추가
    logConsole.innerHTML = logs.map(log => `<div class="log-line">${log}</div>`).join('');
    
    // 최신 메시지로 스크롤
    logConsole.scrollTop = logConsole.scrollHeight;
}

function initChart() {
    const ctx = document.getElementById('forceChart');
    if (!ctx) {
        console.warn("⚠️ Chart canvas not found");
        return;
    }
    
    if (forceChart) {
        forceChart.destroy();
    }
    
    forceChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: chartData.labels,
            datasets: [{
                label: 'Z-Axis Force (N)',
                data: chartData.data,
                borderColor: '#00bcd4',
                backgroundColor: 'rgba(0, 188, 212, 0.1)',
                tension: 0.4,
                fill: true,
                pointRadius: 2,
                pointBackgroundColor: '#00bcd4'
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: true,
            plugins: {
                legend: {
                    labels: {
                        color: '#e0e0e0',
                        font: { size: 12 }
                    }
                }
            },
            scales: {
                y: {
                    beginAtZero: true,
                    ticks: { color: '#888' },
                    grid: { color: 'rgba(255, 255, 255, 0.1)' }
                },
                x: {
                    ticks: { color: '#888' },
                    grid: { color: 'rgba(255, 255, 255, 0.1)' }
                }
            }
        }
    });
}

function addChartData(value) {
    if (!forceChart) return;
    
    const now = new Date();
    const label = now.toLocaleTimeString('ko-KR', { hour12: false });
    
    chartData.labels.push(label);
    chartData.data.push(value);
    
    // 최근 60개만 유지
    if (chartData.labels.length > 60) {
        chartData.labels.shift();
        chartData.data.shift();
    }
    
    forceChart.data.labels = chartData.labels;
    forceChart.data.datasets[0].data = chartData.data;
    forceChart.update();
}

function updateConnectionStatus(connected) {
    const status = document.getElementById('connection-status');
    if (connected) {
        status.textContent = '🟢 Connected';
        status.style.color = '#4caf50';
    } else {
        status.textContent = '🔴 Disconnected';
        status.style.color = '#f44336';
    }
}

function initDashboard() {
    console.log("🚀 Dashboard Initialization Started");
    
    try {
        // 차트 초기화
        initChart();
        console.log("✅ Chart initialized");
        
        // Firebase 리스너 시작
        startFirebaseListener();
        console.log("✅ Firebase listener started");
        
        // 초기 데이터 로드
        loadInitialData();
        console.log("✅ Initial data loaded");
        
    } catch (error) {
        console.error("❌ Dashboard initialization error:", error);
        updateConnectionStatus(false);
    }
}

function loadInitialData() {
    console.log("📥 Loading initial data from Firebase...");
    
    const url = `${FIREBASE_CONFIG.databaseURL}/one.json?auth=${FIREBASE_CONFIG.apiKey}`;
    
    fetch(url)
        .then(response => {
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            return response.json();
        })
        .then(data => {
            console.log("✅ Initial data loaded:", data);
            if (data) {
                lastTimestamp = data.timestamp;
                updateDashboard(data);
            }
        })
        .catch(error => {
            console.error("❌ Error loading initial data:", error);
            updateConnectionStatus(false);
        });
}

// 페이지 로드 시 초기화
document.addEventListener('DOMContentLoaded', initDashboard);
