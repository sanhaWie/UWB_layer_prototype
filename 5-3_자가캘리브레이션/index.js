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

/* ===== 5-3: 자가 캘리브레이션 (좌표 보정) ===== */
function calibrateAnchors(inputAnchors, measuredDists) {
    /* inputAnchors: [{x,y}, {x,y}, {x,y}, {x,y}] (A1~A4)
       measuredDists: { "A1-A2": mm, "A1-A3": mm, ... } (6쌍)
       반환: { corrected: [{x,y},...], errors: {before, after} } */

    const names = ['A1', 'A2', 'A3', 'A4'];
    const pairs = [];
    for (let i = 0; i < 4; i++) {
        for (let j = i + 1; j < 4; j++) {
            const key = names[i] + '-' + names[j];
            if (measuredDists[key] && measuredDists[key] > 0) {
                pairs.push({ i, j, dist: measuredDists[key] / 1000.0 }); /* mm → m */
            }
        }
    }

    if (pairs.length < 3) return null; /* 최소 3쌍 필요 */

    /* 입력 좌표에서의 오차 계산 */
    function calcError(pos) {
        let sum = 0;
        pairs.forEach(p => {
            const dx = pos[p.i].x - pos[p.j].x;
            const dy = pos[p.i].y - pos[p.j].y;
            const calcDist = Math.sqrt(dx * dx + dy * dy);
            const err = calcDist - p.dist;
            sum += err * err;
        });
        return Math.sqrt(sum / pairs.length); /* RMSE (m) */
    }

    const beforeError = calcError(inputAnchors);

    /* 경사 하강법으로 좌표 보정 (A1 고정, A2~A4 조정) */
    let pos = inputAnchors.map(a => ({ x: a.x, y: a.y }));
    const lr = 0.001; /* 학습률 */
    const iterations = 5000;

    for (let iter = 0; iter < iterations; iter++) {
        const grad = pos.map(() => ({ x: 0, y: 0 }));

        pairs.forEach(p => {
            const dx = pos[p.i].x - pos[p.j].x;
            const dy = pos[p.i].y - pos[p.j].y;
            const calcDist = Math.sqrt(dx * dx + dy * dy);
            if (calcDist < 0.001) return;
            const err = calcDist - p.dist;
            const scale = 2 * err / calcDist;

            /* A1(index 0)은 고정 */
            if (p.i !== 0) {
                grad[p.i].x += scale * dx;
                grad[p.i].y += scale * dy;
            }
            if (p.j !== 0) {
                grad[p.j].x -= scale * dx;
                grad[p.j].y -= scale * dy;
            }
        });

        for (let k = 1; k < 4; k++) { /* A1 고정, A2~A4 조정 */
            pos[k].x -= lr * grad[k].x;
            pos[k].y -= lr * grad[k].y;
        }
    }

    const afterError = calcError(pos);

    /* 소수점 3자리 반올림 */
    const corrected = pos.map(p => ({
        x: Math.round(p.x * 1000) / 1000,
        y: Math.round(p.y * 1000) / 1000
    }));

    return {
        corrected,
        errors: {
            before: Math.round(beforeError * 1000), /* mm */
            after: Math.round(afterError * 1000),   /* mm */
        },
        pairs: pairs.map(p => {
            const dx = corrected[p.i].x - corrected[p.j].x;
            const dy = corrected[p.i].y - corrected[p.j].y;
            return {
                pair: names[p.i] + '-' + names[p.j],
                measured: Math.round(p.dist * 1000),
                calculated: Math.round(Math.sqrt(dx*dx + dy*dy) * 1000),
            };
        }),
    };
}

/* ===== 최근 태그 데이터 ===== */
let tagData = {
    dist: [-1, -1, -1, -1],
    neo: { r: 0, g: 0, b: 0 },
    seq: 0,
    lastUpdate: 0,
};

/* ===== 캘리브레이션 결과 ===== */
let calibResults = {}; /* key: "A1-A2", value: dist_mm */

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

/* 캘리브레이션 결과 수신 (포트 8081) */
const calibServer = dgram.createSocket('udp4');
const addrToName = { 0x0010: 'A1', 0x0020: 'A2', 0x0030: 'A3', 0x0040: 'A4' };
const idToName = { 1: 'A1', 2: 'A2', 3: 'A3', 4: 'A4' };

/* 레인징 결과 resolve용 */
let calibResolve = null;

calibServer.on('message', (msg) => {
    if (msg.length < 8) return;
    const cmd = msg.readUInt8(0);
    if (cmd !== 0x50) return; /* CMD_RANGE */
    const anchor_id = msg.readUInt8(1);
    const target = msg.readUInt16LE(2);
    const dist = msg.readFloatLE(4);

    const from = idToName[anchor_id] || '?';
    const to = addrToName[target] || '?';
    console.log(`RANGE: ${from}→${to} = ${dist.toFixed(0)}mm`);

    if (calibResolve) {
        calibResolve({ from, to, dist });
        calibResolve = null;
    }
});

calibServer.bind(8081, () => { console.log('캘리브레이션 수신: 포트 8081'); });

udpServer.on('message', (msg, rinfo) => {
    if (msg.length < 24) return;

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
    <button onclick="startCalib()" style="background:#604000;">앵커간 거리 측정</button>
    <button onclick="correctPos()" style="background:#004060;">좌표 보정</button>
    <div style="margin-top:8px;">LED: <span id="ledColor" style="padding:2px 20px;">-</span> seq: <span id="seq">-</span></div>
    <div id="calibResult" style="margin-top:10px;"></div>
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
    } else if (msg.type === 'calib') {
        showCalib(msg.data);
    } else if (msg.type === 'correction') {
        /* 보정 결과는 correctPos()에서 직접 처리 */
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

async function startCalib() {
    if (!confirm('DS-TWR 캘리브레이션을 시작할까요?')) return;
    document.getElementById('calibResult').innerHTML = '<b>캘리브레이션 진행 중...</b>';
    await fetch('/api/calibrate', { method: 'POST' });
}

function showCalib(data) {
    const pairs = Object.entries(data);
    if (pairs.length === 0) return;
    let html = '<b>캘리브레이션 결과 (' + pairs.length + '/6쌍):</b><br>';
    pairs.forEach(([k, v]) => { html += k + ': ' + v.toFixed(0) + 'mm<br>'; });
    document.getElementById('calibResult').innerHTML = html;
}

async function correctPos() {
    const res = await fetch('/api/correct', { method: 'POST' });
    const data = await res.json();
    if (data.error) { alert(data.error); return; }
    let html = '<b>좌표 보정 결과:</b><br>';
    html += '보정 전 RMSE: ' + data.errors.before + 'mm<br>';
    html += '보정 후 RMSE: ' + data.errors.after + 'mm<br><br>';
    data.pairs.forEach(p => {
        html += p.pair + ': 측정=' + p.measured + 'mm, 계산=' + p.calculated + 'mm<br>';
    });
    html += '<br><b>보정된 좌표:</b><br>';
    data.corrected.forEach((pos, i) => {
        html += 'A' + (i+1) + ': (' + pos.x + ', ' + pos.y + ')<br>';
    });
    document.getElementById('calibResult').innerHTML = html;
    /* 입력 필드 업데이트 */
    data.corrected.forEach((pos, i) => {
        document.getElementById('a'+(i+1)+'x').value = pos.x;
        document.getElementById('a'+(i+1)+'y').value = pos.y;
    });
    readAnchorsFromInputs();
    draw();
}

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
    } else if (req.method === 'POST' && req.url === '/api/calibrate') {
        /* UDP 순차 캘리브레이션: 태그 STOP → 앵커에 UDP 레인징 명령 → 결과 수신 → 태그 RESUME */
        calibResults = {};
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ ok: true, msg: 'started' }));

        const anchorIPs = ['192.168.45.146', '192.168.45.134', '192.168.45.122', '192.168.45.183'];
        const anchorAddrs = [0x0010, 0x0020, 0x0030, 0x0040];
        const anchorNames = ['A1', 'A2', 'A3', 'A4'];
        const CMD_STOP = 0x30, CMD_RESUME = 0x31, CMD_RANGE = 0x50;
        const tagIP = '192.168.45.22';

        (async () => {
            /* 1. 태그 STOP */
            const stopSock = dgram.createSocket('udp4');
            stopSock.send(Buffer.from([CMD_STOP]), 8082, tagIP, () => stopSock.close());
            console.log('태그 STOP');
            await new Promise(r => setTimeout(r, 1000));

            /* 2. 6쌍 순차 레인징 */
            const cmdSock = dgram.createSocket('udp4');

            for (let i = 0; i < 4; i++) {
                for (let j = i + 1; j < 4; j++) {
                    const key = anchorNames[i] + '-' + anchorNames[j];
                    const targetHi = (anchorAddrs[j] >> 8) & 0xFF;
                    const targetLo = anchorAddrs[j] & 0xFF;
                    console.log(`캘리브: ${key}`);

                    let success = false;
                    for (let attempt = 0; attempt < 3; attempt++) {
                        /* 레인징 명령 UDP 전송 */
                        cmdSock.send(Buffer.from([CMD_RANGE, targetHi, targetLo]), 8082, anchorIPs[i]);

                        /* 결과 대기 (최대 5초) */
                        const result = await new Promise((resolve) => {
                            calibResolve = resolve;
                            setTimeout(() => {
                                if (calibResolve === resolve) {
                                    calibResolve = null;
                                    resolve(null);
                                }
                            }, 5000);
                        });

                        if (result && result.dist > 0) {
                            calibResults[key] = result.dist;
                            console.log(`  결과: ${key} = ${result.dist.toFixed(0)}mm (시도 ${attempt+1})`);
                            success = true;
                            break;
                        }
                        console.log(`  실패: ${key} (시도 ${attempt+1})`);
                        await new Promise(r => setTimeout(r, 500));
                    }
                    if (!success) calibResults[key] = -1;
                    broadcast({ type: 'calib', data: calibResults });
                    await new Promise(r => setTimeout(r, 300));
                }
            }
            cmdSock.close();

            /* 3. 태그 RESUME */
            const resumeSock = dgram.createSocket('udp4');
            resumeSock.send(Buffer.from([CMD_RESUME]), 8082, tagIP, () => resumeSock.close());
            console.log('태그 RESUME');

            console.log('캘리브레이션 완료:', calibResults);
            broadcast({ type: 'calib_done', data: calibResults });
        })();
    } else if (req.method === 'GET' && req.url === '/api/calib') {
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify(calibResults));
    } else if (req.method === 'POST' && req.url === '/api/correct') {
        /* 5-3: 좌표 보정 실행 */
        const inputPos = Object.values(anchors).map(a => ({ x: a.x, y: a.y }));
        const result = calibrateAnchors(inputPos, calibResults);
        if (result) {
            /* 보정된 좌표를 앵커에 적용 */
            const names = ['A1', 'A2', 'A3', 'A4'];
            result.corrected.forEach((pos, i) => {
                anchors[names[i]] = { x: pos.x, y: pos.y, set: true };
            });
            broadcast({ type: 'anchors', data: result.corrected });
            broadcast({ type: 'correction', data: result });
            console.log('좌표 보정 완료:', result);
            res.writeHead(200, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify(result));
        } else {
            res.writeHead(400, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ error: '캘리브레이션 데이터 부족 (최소 3쌍)' }));
        }
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
