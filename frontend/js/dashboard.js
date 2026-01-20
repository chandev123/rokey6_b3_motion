// dashboard.js - Simple Dashboard for Robot Control

const FIREBASE_URL = "https://rokey-b-3-default-rtdb.firebaseio.com";
const API_KEY = "AIzaSyCVaEaIp1lyqLlvKR7rBFDpLNyp3Iavx48";

let lastUpdateTime = null;
let updateCount = 0;
let errorCount = 0;

// ========== Update Functions ==========
function updateStatus(status) {
    const statusBox = document.getElementById('status-box');
    if (statusBox) {
        statusBox.className = 'status-indicator ' + status;
        statusBox.textContent = status;
    }
}

function updateJointValues(joints) {
    if (!Array.isArray(joints) || joints.length < 6) {
        console.warn('⚠️ Invalid joints data:', joints);
        return;
    }
    
    console.log('📍 Updating joint values:', joints);
    
    for (let i = 0; i < 6; i++) {
        const angle = joints[i];
        
        // Update gauge
        const gaugeFill = document.getElementById(`joint-gauge-${i}`);
        if (gaugeFill) {
            const percentage = ((angle + 180) / 360) * 100;
            gaugeFill.style.width = Math.max(0, Math.min(100, percentage)) + '%';
        }
        
        // Update value text
        const gaugeValue = document.getElementById(`joint-value-${i}`);
        if (gaugeValue) {
            gaugeValue.textContent = angle.toFixed(1) + '°';
        }
    }
}

function updateDebugInfo(status, timestamp) {
    const debugStatus = document.getElementById('debug-status');
    const debugTimestamp = document.getElementById('debug-timestamp');
    
    if (debugStatus) {
        const now = new Date().toLocaleTimeString('ko-KR');
        debugStatus.innerHTML = `
            <div>✅ Last Update: ${now}</div>
            <div>📈 Updates: ${updateCount} | ❌ Errors: ${errorCount}</div>
            <div>Status: ${status}</div>
        `;
    }
    
    if (debugTimestamp && timestamp) {
        const updateDate = new Date(timestamp).toLocaleString('ko-KR');
        debugTimestamp.textContent = `Server time: ${updateDate}`;
    }
}

// ========== Firebase Data Update ==========
function pollRobotData() {
    const dataUrl = `${FIREBASE_URL}/one.json?auth=${API_KEY}`;
    
    console.log('🔄 Polling Firebase data...');
    
    fetch(dataUrl, { method: 'GET' })
        .then(response => {
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}`);
            }
            return response.json();
        })
        .then(data => {
            if (!data) {
                console.log('⚠️ No data received from Firebase');
                updateDebugInfo('No data', null);
                return;
            }
            
            console.log('✅ Firebase data received:', data);
            updateCount++;
            lastUpdateTime = new Date();
            
            // Update status
            if (data.status) {
                updateStatus(data.status);
                console.log('📌 Status:', data.status);
            }
            
            // Update joints
            if (data.joints) {
                updateJointValues(data.joints);
                console.log('📍 Joints:', data.joints);
            } else {
                console.log('⚠️ No joints data in Firebase');
            }
            
            // Update connection status
            const connStatus = document.getElementById('connection-status');
            if (connStatus) {
                connStatus.textContent = '🟢 Connected';
                connStatus.className = 'status-connected';
            }
            
            updateDebugInfo(data.status || 'WAITING', data.timestamp);
        })
        .catch(error => {
            errorCount++;
            console.error('❌ Polling error:', error);
            
            const connStatus = document.getElementById('connection-status');
            if (connStatus) {
                connStatus.textContent = '🔴 Disconnected';
                connStatus.className = 'status-disconnected';
            }
            
            updateDebugInfo('Connection Error', null);
        });
}

// ========== Control Commands ==========
function sendCommand(command) {
    console.log('🎮 Sending command:', command);
    
    const commandUrl = `${FIREBASE_URL}/one/command.json?auth=${API_KEY}`;
    
    const commandData = {
        cmd: command,
        timestamp: new Date().toISOString()
    };
    
    fetch(commandUrl, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(commandData)
    })
    .then(response => {
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        return response.json();
    })
    .then(data => {
        console.log('✅ Command sent successfully:', command, data);
    })
    .catch(error => {
        console.error('❌ Error sending command:', error);
    });
}

// ========== Test Data Function ==========
function createTestData() {
    console.log('🧪 Creating test data in Firebase...');
    
    const testDataUrl = `${FIREBASE_URL}/one.json?auth=${API_KEY}`;
    
    const testData = {
        status: "RUNNING",
        joints: [45.5, -30.2, 60.1, 15.3, -45.6, 90.0],
        timestamp: new Date().toISOString()
    };
    
    fetch(testDataUrl, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(testData)
    })
    .then(response => response.json())
    .then(data => {
        console.log('✅ Test data created:', data);
        pollRobotData(); // 즉시 데이터 갱신
    })
    .catch(error => {
        console.error('❌ Error creating test data:', error);
    });
}

// ========== Button Event Listeners ==========
const btnStart = document.getElementById('btn-start');
const btnStop = document.getElementById('btn-stop');
const btnEmergencyStop = document.getElementById('btn-emergency-stop');

if (btnStart) {
    btnStart.addEventListener('click', function() {
        console.log('▶️ START button clicked');
        sendCommand('START');
        updateStatus('RUNNING');
    });
}

if (btnStop) {
    btnStop.addEventListener('click', function() {
        console.log('⏸️ STOP button clicked');
        sendCommand('STOP');
        updateStatus('WAITING');
    });
}

if (btnEmergencyStop) {
    btnEmergencyStop.addEventListener('click', function() {
        console.log('🛑 EMERGENCY STOP button clicked');
        if (confirm('정말로 긴급 정지를 실행하시겠습니까?')) {
            sendCommand('EMERGENCY_STOP');
            updateStatus('EMERGENCY');
        }
    });
}

// ========== Initialize ==========
console.log('🚀 Dashboard initializing...');

const connStatus = document.getElementById('connection-status');
if (connStatus) {
    connStatus.textContent = '🟢 Connecting...';
    connStatus.className = 'status-disconnected';
}

// Initial poll
pollRobotData();

// Start polling robot data every 500ms
setInterval(pollRobotData, 500);

// Make test function available in console
window.testRobotData = createTestData;

console.log('✅ Dashboard initialized');
console.log('💡 Tip: Type testRobotData() in console to create test data');