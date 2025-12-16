const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const rclnodejs = require('rclnodejs');
const path = require('path');

const app = express();
const port = process.env.PORT || 8080;
const std_msgs = rclnodejs.require('std_msgs').msg;

// Serve static files
app.use(express.static(path.join(__dirname)));
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});

const server = http.createServer(app);
const wss = new WebSocket.Server({ server });
let wsClients = [];

/* =======================
   WebSocket Handler
======================= */
wss.on('connection', (ws) => {
    console.log("[WS] GUI connected");
    wsClients.push(ws);

    ws.on('message', (msg) => {
        const msgStr = msg.toString();
        console.log("[WS → ROS2] Received:", msgStr);

        // Echo back to GUI (debug)
        wsClients.forEach(client => {
            if (client.readyState === WebSocket.OPEN) {
                client.send(`[Echo from /gui_events]: ${msgStr}`);
            }
        });

        // Publish raw GUI events
        if (global.guiPublisher) {
            global.guiPublisher.publish({ data: msgStr });
        }

        // Publish joint commands if message matches format
        try {
            const parsed = JSON.parse(msgStr);

            if (parsed.type === "joint_cmd" && global.jointPublisher) {
                const j = parsed.data;

                const rosMsg = new std_msgs.Float64MultiArray();
                rosMsg.data = [
                    j.joint1,
                    j.joint2,
                    j.joint3,
                    j.joint4,
                    j.joint5,
                    j.joint6
                ];

                global.jointPublisher.publish(rosMsg);
                console.log("[ROS2] Published joint command:", rosMsg.data);
            }
        } catch (e) {
            // Ignore non-JSON messages
        }
    });

    ws.on('close', () => {
        console.log("[WS] GUI disconnected");
        wsClients = wsClients.filter(c => c !== ws);
    });
});

/* =======================
   ROS2 Initialization
======================= */
async function initROS2() {
    await rclnodejs.init();

    const node = new rclnodejs.Node('openmct_ros2_bridge');

    // Expose node globally if needed later
    global.rosNode = node;

    // GUI event publisher
    global.guiPublisher = node.createPublisher(
        'std_msgs/msg/String',
        'gui_events'
    );

    // Joint command publisher
    global.jointPublisher = node.createPublisher(
        'std_msgs/msg/Float64MultiArray',
        'arm/joint_commands'
    );

    console.log("[ROS2] Publishers ready:");
    console.log("  • /gui_events (String)");
    console.log("  • /arm/joint_commands (Float64MultiArray)");

    // Subscribe to feedback from robot and forward to GUI
    node.createSubscription(
        'std_msgs/msg/String',
        'feedback_from_robot',
        (msg) => {
            wsClients.forEach(ws => {
                if (ws.readyState === WebSocket.OPEN) {
                    ws.send(msg.data);
                }
            });
        }
    );

    rclnodejs.spin(node);
}

initROS2().catch(err => console.error(err));

/* =======================
   Start Server
======================= */
server.listen(port, () => {
    console.log(`\n🚀 GUI running at: http://localhost:${port}`);
    console.log("🟢 ROS2 bridge active (WebSocket on same port)");
});
