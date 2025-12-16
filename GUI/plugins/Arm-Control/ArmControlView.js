// ArmControlView.js
class ArmControlView {
    constructor(container, openmct, wsUrl = "ws://localhost:8080") {
        this.container = container;
        this.openmct = openmct;

        // ======== Joint State ========
        this.jointValues = {
            joint1: 0,
            joint2: 0,
            joint3: 0,
            joint4: 0,
            joint5: 0,
            joint6: 0
        };
        this.mode = "FK"; // FK or IK

        // ======== WebSocket ========
        this.ws = new WebSocket(wsUrl);
        this.ws.onopen = () => console.log("[WS] Connected to Node.js WebSocket server");
        this.ws.onmessage = (msg) => this.handleWSMessage(msg.data);
        this.ws.onclose = () => console.log("[WS] Disconnected");

        // ======== Render HTML inside container ========
        this.container.innerHTML = this.getHTML();

        // ======== Bind Elements ========
        this.bindElements();
    }

    render() {
        this.updateJointStateDisplay();
    }

    destroy() {
        // Clean up any event listeners if needed
        document.removeEventListener("keydown", this.boundKeyboardHandler);
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.close();
        }
    }

    getHTML() {
        return `
        <div class="arm-control-container">
            <div class="status-bar">
                <div id="joystickStatusDot" class="status-dot"></div>
                <div id="joystickStatus">Disconnected</div>
                <button id="modeSwitchButton">Switch to IK Mode</button>
            </div>

            <div class="controls-grid">
                <div class="joint-control">
                    <h2 class="section-header">Joint Control (FK Mode)</h2>
                    <p>Use sliders or input boxes to command each joint.</p>
                    ${[1,2,3,4,5,6].map(i => `
                        <div class="joint-slider-container">
                            <span id="joint${i}Vel">J${i}: 0</span>
                            <input type="number" id="joint${i}Input" class="joint-value-input" min="0" max="180" value="0">
                            <input type="range" id="joint${i}Slider" min="0" max="180" value="0">
                        </div>
                    `).join('')}
                    <p>Presets:</p>
                    <div class="preset-buttons">
                        <button id="presetButton1" class="preset-button">Pre-Pick</button>
                        <button id="presetButton2" class="preset-button">Home</button>
                        <button id="presetButton3" class="preset-button">Rock Storage</button>
                    </div>
                </div>

                <div class="joint-state-section">
                    <h2 class="section-header">Live Joint States</h2>
                    <div class="joint-state-grid">
                        ${[1,2,3,4,5,6].map(i => `<div class="joint-state-block" id="joint${i}State">J${i}: --</div>`).join('')}
                    </div>
                </div>
            </div>
        </div>
        `;
    }

    bindElements() {
        // ======== Sliders & Inputs ========
        for (let i = 1; i <= 6; i++) {
            const slider = this.container.querySelector(`#joint${i}Slider`);
            const input = this.container.querySelector(`#joint${i}Input`);
            const stateDisplay = this.container.querySelector(`#joint${i}Vel`);

            if (!slider || !input || !stateDisplay) {
                console.warn(`Joint ${i} elements not found in container`);
                continue;
            }

            slider.addEventListener("input", e => this.updateJointFromSlider(i, e.target.value));
            input.addEventListener("change", e => this.updateJointFromInput(i, e.target.value));

            this[`joint${i}StateDisplay`] = stateDisplay;
        }

        // ======== Presets ========
        this.container.querySelector("#presetButton1").addEventListener("click", () => this.applyPreset([0, 45, 30, 0, 0, 0]));
        this.container.querySelector("#presetButton2").addEventListener("click", () => this.applyPreset([0, 0, 0, 0, 0, 0]));
        this.container.querySelector("#presetButton3").addEventListener("click", () => this.applyPreset([90, 90, 0, 0, 0, 0]));

        // ======== Mode Toggle ========
        this.container.querySelector("#modeSwitchButton").addEventListener("click", () => {
            this.mode = this.mode === "FK" ? "IK" : "FK";
            this.container.querySelector("#modeSwitchButton").innerText = `Switch to ${this.mode === "FK" ? "IK" : "FK"} Mode`;
            console.log("[Mode] Switched to", this.mode);
        });

        // ======== Keyboard Shortcuts ========
        this.boundKeyboardHandler = e => this.handleKeyboard(e);
        document.addEventListener("keydown", this.boundKeyboardHandler);
    }

    // ======== Update Functions ========
    updateJointFromSlider(jointNum, value) {
        this.jointValues[`joint${jointNum}`] = Number(value);
        this.container.querySelector(`#joint${jointNum}Input`).value = value;
        this.sendJointValues();
        this.updateJointStateDisplay();
    }

    updateJointFromInput(jointNum, value) {
        this.jointValues[`joint${jointNum}`] = Number(value);
        this.container.querySelector(`#joint${jointNum}Slider`).value = value;
        this.sendJointValues();
        this.updateJointStateDisplay();
    }

    applyPreset(values) {
        for (let i = 1; i <= 6; i++) {
            this.jointValues[`joint${i}`] = values[i-1];
            this.container.querySelector(`#joint${i}Slider`).value = values[i-1];
            this.container.querySelector(`#joint${i}Input`).value = values[i-1];
        }
        this.sendJointValues();
        this.updateJointStateDisplay();
    }

    updateJointStateDisplay() {
        for (let i = 1; i <= 6; i++) {
            if (this[`joint${i}StateDisplay`]) {
                this[`joint${i}StateDisplay`].innerText = `J${i}: ${this.jointValues[`joint${i}`]}`;
            }
        }
    }

    // ======== WebSocket ========
    sendJointValues() {
        if (this.ws.readyState === WebSocket.OPEN) {
            const msg = {
                type: "joint_cmd",
                mode: this.mode,
                data: this.jointValues
            };
            console.log("Sending:", msg);
            this.ws.send(JSON.stringify(msg));
        }
    }

    handleWSMessage(data) {
        console.log("[WS] Received from ROS2:", data);
    }

    // ======== Keyboard ========
    handleKeyboard(e) {
        const step = 1;
        switch(e.key.toLowerCase()) {
            case "q": this.incrementJoint(1, -step); break;
            case "w": this.incrementJoint(1, step); break;
            case "a": this.incrementJoint(2, -step); break;
            case "s": this.incrementJoint(2, step); break;
            case "z": this.incrementJoint(3, -step); break;
            case "x": this.incrementJoint(3, step); break;
            case "e": this.incrementJoint(4, -step); break;
            case "r": this.incrementJoint(4, step); break;
            case "d": this.incrementJoint(5, -step); break;
            case "f": this.incrementJoint(5, step); break;
            case "c": this.incrementJoint(6, -step); break;
            case "v": this.incrementJoint(6, step); break;
        }
    }

    incrementJoint(jointNum, delta) {
        const newVal = this.jointValues[`joint${jointNum}`] + delta;
        this.updateJointFromInput(jointNum, newVal);
    }
}

// Expose globally for plugin.js
window.ArmControlView = ArmControlView;