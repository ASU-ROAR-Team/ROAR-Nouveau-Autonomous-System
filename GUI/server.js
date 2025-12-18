//server.js
const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const rclnodejs = require('rclnodejs');

const app = express();
const server = http.createServer(app);
const path = require('path');

app.use(express.static(__dirname));

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});

const wss = new WebSocket.Server({ server });

let jointPublisher;

/* ===== WebSocket ===== */
wss.on('connection', ws => {
    console.log('[WS] Client connected');

    ws.on('message', msg => {
        try {
            const parsed = JSON.parse(msg.toString());
            if (parsed.type !== 'joint_cmd') return;

            const arr = parsed.data.map(Number);
            if (arr.length !== 6 || arr.some(v => isNaN(v))) return;

            jointPublisher.publish({ data: arr });
            console.log('[ROS2] Published:', arr);
        } catch (e) {
            console.error('Bad WS message');
        }
    });
});

/* ===== ROS2 ===== */
async function init() {
    await rclnodejs.init();
    const node = new rclnodejs.Node('openmct_ros2_bridge');

    jointPublisher = node.createPublisher(
        'std_msgs/msg/Float64MultiArray',
        'arm/joint_commands'
    );

    rclnodejs.spin(node);
    server.listen(8080, () =>
        console.log('🚀 http://localhost:8080')
    );
}

init();
