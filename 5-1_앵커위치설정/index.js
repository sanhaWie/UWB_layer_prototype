/**
 * 계층 5~7: UWB RTLS 서버
 *
 * - 웹 UI: Canvas에서 앵커 위치 설정
 * - UDP 수신: 태그 거리 데이터
 * - WebSocket: 브라우저에 실시간 데이터 전달
 */
const http = require('http');
const dgram = require('dgram');
const fs = require('fs');
const path = require('path');

const HTTP_PORT = 3000;
const UDP_PORT = 8080;

/* ===== 앵커 위치 저장 ===== */
let anchors = {
    A1: { x: 0, y: 0, set: false },
    A2: { x: 0, y: 0, set: false },
    A3: { x: 0, y: 0, set: false },
    A4: { x: 0, y: 0, set: false },
};

/* ===== 최근 태그 데이터 ===== */
let tagData = {
    dist: [-1, -1, -1, -1],
    neo: { r: 0, g: 0, b: 0 },
    seq: 0,
    lastUpdate: 0,
};

/* ===== WebSocket 클라이언트 ===== */
let wsClients = [];

function broadcast(data) {
    const msg = JSON.stringify(data);
    wsClients = wsClients.filter(ws => {
        if (ws.readyState === 1) { /* WebSocket.OPEN */
            try { ws.send(msg); return true; }
            catch { return false; }
        }
        return false;
    });
}

/* ===== UDP 수신 ===== */
const udpServer = dgram.createSocket('udp4');

udpServer.on('message', (msg, rinfo) => {
    if (msg.length < 24) return;
    console.log(`UDP: ${msg.length}바이트 from ${rinfo.address}`);

    tagData.dist = [
        msg.readFloatLE(8),
        msg.readFloatLE(12),
        msg.readFloatLE(16),
        msg.readFloatLE(20),
    ];
    tagData.neo = {
        r: msg.readUInt8(2),
        g: msg.readUInt8(3),
        b: msg.readUInt8(4),
    };
    tagData.seq = msg.readUInt16LE(6);
    tagData.lastUpdate = Date.now();

    broadcast({ type: 'tag', data: tagData });
});

udpServer.bind(UDP_PORT, () => {
    console.log(`UDP 수신: 포트 ${UDP_PORT}`);
});

/* ===== HTTP 서버 ===== */
const htmlPage = `<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>UWB RTLS</title>
<style>
    body { font-family: sans-serif; margin: 20px; background: #1a1a2e; color: #eee; }
    h2 { color: #0ff; }
    canvas { border: 1px solid #444; background: #16213e; cursor: crosshair; border-radius: 8px; }
    .info { margin: 10px 0; font-family: monospace; font-size: 14px; }
    button { padding: 8px 16px; margin: 5px; background: #0f3460; color: #eee;
             border: 1px solid #0ff; border-radius: 4px; cursor: pointer; }
    button:hover { background: #0ff; color: #000; }
    .status { padding: 10px; background: #0a0a23; border-radius: 4px; margin: 10px 0; }
    table { border-collapse: collapse; margin: 10px 0; }
    td, th { border: 1px solid #444; padding: 6px 12px; }
</style>
</head><body>

<h2>UWB 실내 측위 시스템</h2>

<div>
    <b>방 크기 (m):</b>
    <input id="roomW" type="number" value="4" min="1" max="20" style="width:50px"> x
    <input id="roomH" type="number" value="4" min="1" max="20" style="width:50px">
</div>

<div style="display:flex; gap:20px; margin-top:10px;">
<div>
    <table>
        <tr><th>앵커</th><th>X (m)</th><th>Y (m)</th><th>거리 (mm)</th></tr>
        <tr><td>A1</td>
            <td><input id="a1x" type="number" step="0.01" value="0" style="width:60px"></td>
            <td><input id="a1y" type="number" step="0.01" value="0" style="width:60px"></td>
            <td id="d1">-</td></tr>
        <tr><td>A2</td>
            <td><input id="a2x" type="number" step="0.01" value="1" style="width:60px"></td>
            <td><input id="a2y" type="number" step="0.01" value="0" style="width:60px"></td>
            <td id="d2">-</td></tr>
        <tr><td>A3</td>
            <td><input id="a3x" type="number" step="0.01" value="0" style="width:60px"></td>
            <td><input id="a3y" type="number" step="0.01" value="1" style="width:60px"></td>
            <td id="d3">-</td></tr>
        <tr><td>A4</td>
            <td><input id="a4x" type="number" step="0.01" value="1" style="width:60px"></td>
            <td><input id="a4y" type="number" step="0.01" value="1" style="width:60px"></td>
            <td id="d4">-</td></tr>
    </table>
    <button onclick="saveAnchors()">앵커 위치 저장</button>
    <div style="margin-top:8px;">LED: <span id="ledColor" style="padding:2px 20px;">-</span> seq: <span id="seq">-</span></div>
</div>
<div>
    <canvas id="canvas" width="500" height="500"></canvas>
</div>
</div>

<script>
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
let anchors = [];
let tagPos = null;
let tagDist = [-1,-1,-1,-1];

function getRoom() {
    return {
        w: parseFloat(document.getElementById('roomW').value) || 4,
        h: parseFloat(document.getElementById('roomH').value) || 4
    };
}

function m2px(mx, my) {
    const room = getRoom();
    const margin = 50;
    const sx = (canvas.width - 2*margin) / room.w;
    const sy = (canvas.height - 2*margin) / room.h;
    return { x: margin + mx * sx, y: canvas.height - margin - my * sy };
}

function readAnchorsFromInputs() {
    anchors = [];
    for (let i = 1; i <= 4; i++) {
        anchors.push({
            x: parseFloat(document.getElementById('a'+i+'x').value) || 0,
            y: parseFloat(document.getElementById('a'+i+'y').value) || 0
        });
    }
}

async function saveAnchors() {
    readAnchorsFromInputs();
    const res = await fetch('/api/anchors', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(anchors)
    });
    const data = await res.json();
    alert(data.ok ? '저장 완료!' : '저장 실패');
    draw();
}

/* 입력값 변경 시 Canvas 업데이트 */
document.querySelectorAll('input[type=number]').forEach(el => {
    el.addEventListener('change', () => { readAnchorsFromInputs(); draw(); });
});

function draw() {
    const room = getRoom();
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    /* 그리드 */
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 0.5;
    for (let x = 0; x <= room.w; x++) {
        const p = m2px(x, 0);
        const p2 = m2px(x, room.h);
        ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
    }
    for (let y = 0; y <= room.h; y++) {
        const p = m2px(0, y);
        const p2 = m2px(room.w, y);
        ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
    }

    /* 축 라벨 */
    ctx.fillStyle = '#888'; ctx.font = '12px monospace';
    for (let x = 0; x <= room.w; x++) {
        const p = m2px(x, 0);
        ctx.fillText(x + 'm', p.x - 8, p.y + 16);
    }
    for (let y = 0; y <= room.h; y++) {
        const p = m2px(0, y);
        ctx.fillText(y + 'm', p.x - 40, p.y + 4);
    }

    /* 앵커 */
    const colors = ['#ff4444', '#44ff44', '#4444ff', '#ffff44'];
    const names = ['A1', 'A2', 'A3', 'A4'];
    for (let i = 0; i < anchors.length; i++) {
        const p = m2px(anchors[i].x, anchors[i].y);
        ctx.fillStyle = colors[i];
        ctx.beginPath(); ctx.arc(p.x, p.y, 10, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = '#fff'; ctx.font = 'bold 12px sans-serif';
        ctx.fillText(names[i], p.x - 8, p.y - 14);

        /* 거리 원 */
        if (tagDist[i] > 0) {
            const room = getRoom();
            const margin = 50;
            const sx = (canvas.width - 2*margin) / room.w;
            const r_m = tagDist[i] / 1000;
            ctx.strokeStyle = colors[i] + '55';
            ctx.lineWidth = 1;
            ctx.beginPath(); ctx.arc(p.x, p.y, r_m * sx, 0, Math.PI * 2); ctx.stroke();
        }
    }

    /* 태그 위치 (삼변측량은 계층 6에서) */
    if (tagPos) {
        const p = m2px(tagPos.x, tagPos.y);
        ctx.fillStyle = '#0ff';
        ctx.beginPath(); ctx.arc(p.x, p.y, 8, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = '#fff'; ctx.font = 'bold 12px sans-serif';
        ctx.fillText('TAG', p.x - 12, p.y - 12);
    }
}

/* WebSocket */
const ws = new WebSocket('ws://' + location.host);
ws.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    if (msg.type === 'tag') {
        tagDist = msg.data.dist;
        for (let i = 0; i < 4; i++) {
            document.getElementById('d'+(i+1)).textContent =
                tagDist[i] >= 0 ? tagDist[i].toFixed(0) : 'FAIL';
        }
        document.getElementById('seq').textContent = msg.data.seq;
        const neo = msg.data.neo;
        const ledEl = document.getElementById('ledColor');
        ledEl.style.background = 'rgb('+neo.r+','+neo.g+','+neo.b+')';
        ledEl.textContent = '('+neo.r+','+neo.g+','+neo.b+')';
        draw();
    } else if (msg.type === 'anchors') {
        anchors = msg.data;
        for (let i = 0; i < 4; i++) {
            document.getElementById('a'+(i+1)+'x').value = anchors[i].x;
            document.getElementById('a'+(i+1)+'y').value = anchors[i].y;
        }
        draw();
    }
};

/* 초기 로드 */
fetch('/api/anchors').then(r => r.json()).then(data => {
    if (data && data.length === 4) {
        for (let i = 0; i < 4; i++) {
            document.getElementById('a'+(i+1)+'x').value = data[i].x;
            document.getElementById('a'+(i+1)+'y').value = data[i].y;
        }
        anchors = data;
    }
    readAnchorsFromInputs();
    draw();
});

readAnchorsFromInputs();
draw();
</script>
</body></html>`;

/* ===== HTTP + WebSocket 서버 ===== */
const server_http = http.createServer((req, res) => {
    if (req.method === 'GET' && req.url === '/') {
        res.writeHead(200, { 'Content-Type': 'text/html' });
        res.end(htmlPage);
    } else if (req.method === 'GET' && req.url === '/api/anchors') {
        const data = Object.values(anchors).map(a => a.set ? { x: a.x, y: a.y } : null);
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify(data.every(a => a) ? data : []));
    } else if (req.method === 'POST' && req.url === '/api/anchors') {
        let body = '';
        req.on('data', c => body += c);
        req.on('end', () => {
            try {
                const arr = JSON.parse(body);
                const names = ['A1', 'A2', 'A3', 'A4'];
                arr.forEach((a, i) => {
                    anchors[names[i]] = { x: a.x, y: a.y, set: true };
                });
                console.log('앵커 위치 저장:', anchors);
                broadcast({ type: 'anchors', data: arr });
                res.writeHead(200, { 'Content-Type': 'application/json' });
                res.end(JSON.stringify({ ok: true }));
            } catch (e) {
                res.writeHead(400);
                res.end(JSON.stringify({ ok: false }));
            }
        });
    } else {
        res.writeHead(404);
        res.end('Not found');
    }
});

/* WebSocket (ws 라이브러리) */
const WebSocket = require('ws');
const wss = new WebSocket.Server({ server: server_http });

wss.on('connection', (ws) => {
    wsClients.push(ws);
    ws.on('close', () => { wsClients = wsClients.filter(c => c !== ws); });
    ws.on('error', () => { wsClients = wsClients.filter(c => c !== ws); });

    /* 현재 앵커 위치 전송 */
    const data = Object.values(anchors).map(a => a.set ? { x: a.x, y: a.y } : null);
    if (data.every(a => a)) {
        ws.send(JSON.stringify({ type: 'anchors', data }));
    }
});

server_http.listen(HTTP_PORT, () => {
    console.log(`웹 서버: http://localhost:${HTTP_PORT}`);
    console.log('브라우저에서 열어 앵커 위치를 설정하세요');
});
