class ArmControlView {
    constructor(element) {
        this.container = element;

        // HTML & CSS
        this.container.innerHTML = `
            <style>
                .arm-control-container {
                    display: flex;
                    flex-direction: column;
                    gap: 12px;
                    font-family: Arial, sans-serif;
                    padding: 10px;
                    background-color: #f5f5f5;
                    border-radius: 8px;
                    max-width: 400px;
                }
                .joint-row {
                    display: flex;
                    gap: 20px; /* space between sliders */
                    width: 100%;
                }

                .joint-control {
                    display: flex;
                    align-items: center;
                    gap: 8px;
                }
                .joint-control label {
                    width: 60px;
                }
                .joint-control input[type="range"] {
                    flex: 1;
                }
                .preset-buttons {
                    display: flex;
                    gap: 10px;
                    margin-top: 10px;
                }
                .preset-buttons button {
                    flex: 1;
                    padding: 6px;
                    border: none;
                    border-radius: 4px;
                    background-color: #007bff;
                    color: white;
                    cursor: pointer;
                }
                .preset-buttons button:hover {
                    background-color: #0056b3;
                }
                .status {
                    margin-top: 8px;
                    font-size: 0.9em;
                    color: #333;
                }
            </style>
            <div class="arm-control-container">
    <div class="joint-row">
        <div class="joint-control">
            <label for="joint1_slider">Joint 1</label>
            <input type="range" id="joint1_slider" min="-180" max="180" value="0">
            <input type="number" id="joint1_number" min="-180" max="180" value="0">
        </div>
        <div class="joint-control">
            <label for="joint2_slider">Joint 2</label>
            <input type="range" id="joint2_slider" min="-180" max="180" value="0">
            <input type="number" id="joint2_number" min="-180" max="180" value="0">
        </div>
    </div>

    <div class="joint-row">
        <div class="joint-control">
            <label for="joint3_slider">Joint 3</label>
            <input type="range" id="joint3_slider" min="-180" max="180" value="0">
            <input type="number" id="joint3_number" min="-180" max="180" value="0">
        </div>
        <div class="joint-control">
            <label for="joint4_slider">Joint 4</label>
            <input type="range" id="joint4_slider" min="-180" max="180" value="0">
            <input type="number" id="joint4_number" min="-180" max="180" value="0">
        </div>
    </div>

    <div class="joint-row">
        <div class="joint-control">
            <label for="joint5_slider">Joint 5</label>
            <input type="range" id="joint5_slider" min="-180" max="180" value="0">
            <input type="number" id="joint5_number" min="-180" max="180" value="0">
        </div>
        <div class="joint-control">
            <label for="joint6_slider">Joint 6</label>
            <input type="range" id="joint6_slider" min="-180" max="180" value="0">
            <input type="number" id="joint6_number" min="-180" max="180" value="0">
        </div>
    </div>
</div>


    <div class="preset-buttons">
        <button id="preset_home">Home</button>
        <button id="preset_stretch">Stretch</button>
        <button id="preset_tuck">Tuck</button>
    </div>

    <div class="status" id="arm_status">Connecting to ROS...</div>

    <div class="shortcuts">
        <div class="shortcut-item"><span>Joint 1</span><span>Q(-) / W(+)</span> <span>Joint 2</span><span>A(-) / S(+)</span></div>
        <div class="shortcut-item"><span>Joint 3</span><span>E(-) / R(+)</span> <span>Joint 4</span><span>D(-) / F(+)</span></div>
        <div class="shortcut-item"><span>Joint 5</span><span>T(-) / Y(+)</span> <span>Joint 6</span><span>G(-) / H(+)</span></div>
    </div>
            </div>
        `;

        this.sliders = {};
        this.numberInputs = {};
        this.presets = {};
        this.jointNames = ['joint1','joint2','joint3','joint4','joint5','joint6'];
        this.ros = null;
        this.jointPublisher = null;
        this.jointStateListener = null;

        // Map HTML elements
        this.jointNames.forEach(joint => {
            this.sliders[joint] = document.getElementById(`${joint}_slider`);
            this.numberInputs[joint] = document.getElementById(`${joint}_number`);

            if (this.sliders[joint] && this.numberInputs[joint]) {
                this.sliders[joint].addEventListener('input', () => {
                    this.numberInputs[joint].value = this.sliders[joint].value;
                    this.publishJointStates();
                });
                this.numberInputs[joint].addEventListener('input', () => {
                    this.sliders[joint].value = this.numberInputs[joint].value;
                    this.publishJointStates();
                });
            }
        });

        const presetButtons = document.querySelectorAll('[id^="preset_"]');
        presetButtons.forEach(button => {
            const presetName = button.id.replace('preset_', '');
            this.presets[presetName] = button;
            button.addEventListener('click', () => this.loadPreset(presetName));
        });

        document.addEventListener('keydown', this.handleKeyDown.bind(this));

        // Try connecting to ROS every second
        this.statusElement = document.getElementById('arm_status');
        this.tryConnectROS();
    }

    tryConnectROS() {
        if (typeof ROSLIB === 'undefined') {
            this.statusElement.innerText = 'ROSLIB not loaded!';
            return;
        }

        try {
            this.ros = new ROSLIB.Ros({
                url: 'ws://localhost:9090'
            });

            this.ros.on('connection', () => {
                this.statusElement.innerText = 'Connected to ROS!';
                this.setupROS();
            });

            this.ros.on('error', () => {
                this.statusElement.innerText = 'ROS connection error. Retrying...';
                setTimeout(() => this.tryConnectROS(), 1000);
            });

            this.ros.on('close', () => {
                this.statusElement.innerText = 'ROS connection closed. Retrying...';
                setTimeout(() => this.tryConnectROS(), 1000);
            });
        } catch (err) {
            this.statusElement.innerText = 'ROS init failed. Retrying...';
            setTimeout(() => this.tryConnectROS(), 1000);
        }
    }

    setupROS() {
        // Publishers
        this.jointPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/fk_joint_states',
            messageType: 'sensor_msgs/JointState'
        });

        // Subscribers
        this.jointStateListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/joint_states',
            messageType: 'sensor_msgs/JointState'
        });
        this.jointStateListener.subscribe(this.updateJointStates.bind(this));
    }

    publishJointStates() {
        if (!this.jointPublisher) return;
        const positions = this.jointNames.map(joint => parseFloat(this.sliders[joint].value) * Math.PI / 180.0);
        const jointState = new ROSLIB.Message({
            name: this.jointNames,
            position: positions,
            velocity: [],
            effort: []
        });
        this.jointPublisher.publish(jointState);
    }

    updateJointStates(message) {
        message.name.forEach((joint, i) => {
            if (this.sliders[joint] && this.numberInputs[joint]) {
                const degreeValue = (message.position[i] * 180.0 / Math.PI).toFixed(1);
                this.sliders[joint].value = degreeValue;
                this.numberInputs[joint].value = degreeValue;
            }
        });
    }

    loadPreset(presetName) {
        const presets = {
            home: [0,0,0,0,0,0],
            stretch: [30,45,0,0,0,0],
            tuck: [-30,-45,0,0,0,0]
        };
        if (presets[presetName]) {
            presets[presetName].forEach((val, i) => {
                const joint = this.jointNames[i];
                if (this.sliders[joint] && this.numberInputs[joint]) {
                    this.sliders[joint].value = val;
                    this.numberInputs[joint].value = val;
                }
            });
            this.publishJointStates();
        }
    }

    handleKeyDown(event) {
        const keyMap = {
            'q': { joint: "joint1", delta: -1 },
            'a': { joint: "joint2", delta: -1 },
            'w': { joint: "joint1", delta: 1 },
            's': { joint: "joint2", delta: 1 },
            'e': { joint: "joint3", delta: -1 },
            'd': { joint: "joint4", delta: -1 },
            'r': { joint: "joint3", delta: 1 },
            'f': { joint: "joint4", delta: 1 },
            't': { joint: "joint5", delta: -1 },
            'g': { joint: "joint6", delta: -1 },
            'y': { joint: "joint5", delta: 1 },
            'h': { joint: "joint6", delta: 1 },
        };
        const action = keyMap[event.key];
        if (action && this.sliders[action.joint] && this.numberInputs[action.joint]) {
            let newVal = parseFloat(this.sliders[action.joint].value) + action.delta;
            this.sliders[action.joint].value = newVal;
            this.numberInputs[action.joint].value = newVal;
            this.publishJointStates();
        }
    }
}

// Expose globally
window.ArmControlView = ArmControlView;
