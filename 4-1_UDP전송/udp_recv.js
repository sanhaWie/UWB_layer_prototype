/**
 * 계층 4-3: UDP 수신 서버 (NeoPixel RGB 포함)
 * 패킷: [dev(1), type(1), neo_r(1), neo_g(1), neo_b(1), state(1), seq(2), dist1~4(float×4)] = 24바이트
 */
const dgram = require('dgram');
const server = dgram.createSocket('udp4');

const PORT = 8080;

server.on('message', (msg, rinfo) => {
    if (msg.length < 24) {
        console.log(`[!] 짧은 패킷 (${msg.length}바이트)`);
        return;
    }

    const device_id = msg.readUInt8(0);
    const pkt_type = msg.readUInt8(1);
    const neo_r = msg.readUInt8(2);
    const neo_g = msg.readUInt8(3);
    const neo_b = msg.readUInt8(4);
    const state = msg.readUInt8(5);
    const seq = msg.readUInt16LE(6);
    const dist1 = msg.readFloatLE(8);
    const dist2 = msg.readFloatLE(12);
    const dist3 = msg.readFloatLE(16);
    const dist4 = msg.readFloatLE(20);

    console.log(`[${seq}] A1=${dist1.toFixed(0)} A2=${dist2.toFixed(0)} A3=${dist3.toFixed(0)} A4=${dist4.toFixed(0)} | LED=(${neo_r},${neo_g},${neo_b})`);
});

server.on('listening', () => {
    console.log(`UDP 수신 서버: 포트 ${PORT} (NeoPixel RGB 포함)`);
});

server.bind(PORT);
