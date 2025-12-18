class ArmControlView {
    constructor(container, openmct, wsUrl = "ws://localhost:8080") {
        this.container = container;
        this.openmct = openmct;

        this.mode = "FK";
        this.jointNames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];

        this.jointValues = {
            joint1: 0, joint2: 0, joint3: 0,
            joint4: 0, joint5: 0, joint6: 0
        };

        this.ikValues = { x: 50, y: 0, z: 20, yaw: 0, pitch: 0, roll: 0 };

        /* ===== WebSocket ===== */
        this.ws = new WebSocket(wsUrl);
        this.ws.onopen = () => console.log("[WS] Connected");
        this.ws.onmessage = msg => console.log("[WS] RX:", msg.data);
        this.ws.onclose = () => console.log("[WS] Closed");

        /* ===== ROS ===== */
        this.ros = null;
        this.jointPublisher = null;
        this.posePublisher = null;

        this.sliders = {};
        this.numberInputs = {};
        this.ikInputs = {};
    }

    render() {
        this.container.innerHTML = this.getHTML();
        // Bind the status element to the specific status ID in your CSS logic
        this.statusElement = this.container.querySelector("#joystickStatus");
        this.statusDot = this.container.querySelector(".status-dot");
        
        this.bindElements();
        this.tryConnectROS();
        this.updateModeUI();
    }

    getHTML() {
        return `
            <div class="arm-control-container">
                <h2 class="section-header">Robotic Arm Control</h2>
                
                <div class="status-bar">
                    <div class="status-dot"></div>
                    <div id="joystickStatus">ROS: Disconnected</div>
                    <button id="modeSwitchButton">Switch to IK Mode</button>
                </div>

                <div class="controls-grid">
                    <div id="fk_container" class="joint-control">
                        <p>Adjust individual joint angles manually.</p>
                        ${this.jointNames.map(joint => `
                            <div class="joint-slider-container">
                                <label style="width: 80px;">${joint.toUpperCase()}</label>
                                <input type="range" id="${joint}_slider" min="-180" max="180" value="0">
                                <span id="${joint}_val_display">0°</span>
                                <input type="number" id="${joint}_number" value="0" style="display:none">
                            </div>
                        `).join('')}
                    </div>

                    <div id="ik_container" class="pose-control" style="display: none;">
                        <p>Define the target end-effector position and orientation.</p>
                        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 15px;">
                            ${['x', 'y', 'z', 'yaw', 'pitch', 'roll'].map(k => `
                                <div class="pose-item">
                                    <strong>${k.toUpperCase()}</strong>
                                    <input type="number" id="${k}_input" value="${this.ikValues[k]}" 
                                           style="width: 70px; padding: 5px; border-radius: 4px; border: 1px solid #ddd;">
                                    <span>${k === 'x' || k === 'y' || k === 'z' ? 'cm' : '°'}</span>
                                </div>
                            `).join('')}
                        </div>
                    </div>
                </div>

                <div class="preset-buttons">
                    <button class="preset-button" onclick="window.dispatchEvent(new CustomEvent('arm-home'))">Home Position</button>
                    <button class="preset-button" onclick="window.dispatchEvent(new CustomEvent('arm-rest'))">Rest Position</button>
                </div>
            </div>
        `;
    }
    /* ================= UI Logic ================= */

   updateModeUI() {
        const fkCont = this.container.querySelector('#fk_container');
        const ikCont = this.container.querySelector('#ik_container');
        const btn = this.container.querySelector('#modeSwitchButton');

        if (this.mode === 'FK') {
            fkCont.style.display = 'flex'; // joint-control uses flex
            ikCont.style.display = 'none';
            btn.innerText = 'Switch to IK Mode';
        } else {
            fkCont.style.display = 'none';
            ikCont.style.display = 'flex'; // pose-control uses flex
            btn.innerText = 'Switch to FK Mode';
        }
    }

    bindElements() {
        // Bind FK Sliders and Numbers
        this.jointNames.forEach(joint => {
            const s = this.container.querySelector(`#${joint}_slider`);
            const n = this.container.querySelector(`#${joint}_number`);

            if (s && n) {
                this.sliders[joint] = s;
                this.numberInputs[joint] = n;

                const handler = val => {
    val = Number(val);
    this.jointValues[joint] = val;
    s.value = val;
    n.value = val;
    // Add this line to update the visual text:
    this.container.querySelector(`#${joint}_val_display`).innerText = `${val}°`;

    if (this.mode === 'FK') this.publishJointStates();
    this.sendWSUpdate();
};
                s.oninput = () => handler(s.value);
                n.oninput = () => handler(n.value);
            }
        });

        // Bind IK Inputs
        ['x', 'y', 'z', 'yaw', 'pitch', 'roll'].forEach(k => {
            const el = this.container.querySelector(`#${k}_input`);
            if (el) {
                this.ikInputs[k] = el;
                el.oninput = () => {
                    this.ikValues[k] = Number(el.value);
                    if (this.mode === 'IK') this.publishPose();
                    this.sendWSUpdate();
                };
            }
        });

        // Mode Switch Button
        const switchBtn = this.container.querySelector('#modeSwitchButton');
        if (switchBtn) {
            switchBtn.onclick = () => {
                this.mode = this.mode === 'FK' ? 'IK' : 'FK';
                this.updateModeUI();
            };
        }
    }

    /* ================= WS ================= */

    sendWSUpdate() {
        if (this.ws.readyState !== WebSocket.OPEN) return;

        const data = this.mode === 'FK'
            ? Object.values(this.jointValues)
            : [this.ikValues.x, this.ikValues.y, this.ikValues.z, this.ikValues.yaw, this.ikValues.pitch, this.ikValues.roll];

        if (data.some(v => isNaN(v))) return;

        this.ws.send(JSON.stringify({
            type: 'joint_cmd',
            mode: this.mode,
            data
        }));
    }

    /* ================= ROS ================= */

    tryConnectROS() {
        if (typeof ROSLIB === 'undefined') {
            if (this.statusElement) this.statusElement.innerText = 'ROSLIB not loaded';
            return;
        }

        this.ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' }); // Note: standard Rosbridge port is 9090

        this.ros.on('connection', () => {
    this.statusDot.classList.add('connected');
    this.statusDot.classList.remove('error');
    this.statusElement.innerText = 'ROS: Connected';
    this.setupROS();
});

        this.ros.on('error', e => {
            console.error('[ROS] Error', e);
            if (this.statusElement) this.statusElement.innerText = 'ROS: Connection Error';
        });
        
        this.ros.on('close', () => {
            if (this.statusElement) this.statusElement.innerText = 'ROS: Connection Closed';
        });
    }

    setupROS() {
        this.jointPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/fk_joint_states',
            messageType: 'sensor_msgs/JointState'
        });

        this.posePublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/ik_target_pose',
            messageType: 'std_msgs/Float64MultiArray'
        });
    }

    publishJointStates() {
        if (!this.jointPublisher) return;
        const pos = this.jointNames.map(j => this.jointValues[j] * Math.PI / 180);
        this.jointPublisher.publish({
            header: { stamp: { secs: 0, nsecs: 0 }, frame_id: '' },
            name: this.jointNames,
            position: pos
        });
    }

    publishPose() {
        if (!this.posePublisher) return;
        const arr = [this.ikValues.x, this.ikValues.y, this.ikValues.z, this.ikValues.yaw, this.ikValues.pitch, this.ikValues.roll].map(Number);
        if (arr.some(v => isNaN(v))) return;
        this.posePublisher.publish({ data: arr });
    }
    
    destroy() {
        if (this.ws) this.ws.close();
        if (this.ros) this.ros.close();
    }
}

window.ArmControlView = ArmControlView;