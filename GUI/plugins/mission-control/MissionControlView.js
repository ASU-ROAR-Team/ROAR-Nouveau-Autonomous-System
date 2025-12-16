// src/plugins/mission-control/MissionControlView.js

(function () {
    'use strict';

    function MissionControlView(element, openmct) {
        this.element = element;
        this.openmct = openmct;
        this.ros = null;
        this.missionService = null;
        this.currentState = 'IDLE';
        this.activeMission = '';
        this.isProcessing = false;
        
        // Updated: Added 'Drilling' to the list, placed before 'Teleoperation'
        this.missions = ['Navigation', 'Sampling', 'Maintenance', 'Drilling', 'Teleoperation'];
        
        this.initROS();
    }

    MissionControlView.prototype.initROS = function() {
        try {
            // Initialize ROS connection (assuming rosbridge_server is running on localhost:9090)
            this.ros = new ROSLIB.Ros({
                url: 'ws://localhost:9090'
            });

            this.ros.on('connection', () => {
                console.log('Mission Control: Connected to ROS');
                this.updateConnectionStatus(true);
            });

            this.ros.on('error', (error) => {
                console.error('Mission Control: ROS connection error:', error);
                this.updateConnectionStatus(false);
            });

            this.ros.on('close', () => {
                console.log('Mission Control: ROS connection closed');
                this.updateConnectionStatus(false);
            });

            // Initialize mission control service
            this.missionService = new ROSLIB.Service({
                ros: this.ros,
                name: '/mission_control',
                // Change the serviceType to the correct package and service name
                serviceType: 'roar_msgs/MissionControl' 
            });

            // Subscribe to rover status
            this.statusSubscriber = new ROSLIB.Topic({
                ros: this.ros,
                name: '/rover_status',
                // Change the messageType to the correct package and message name
                messageType: 'roar_msgs/RoverStatus' 
            });

            this.statusSubscriber.subscribe((message) => {
                this.updateRoverStatus(message);
            });

        } catch (error) {
            console.error('Mission Control: Failed to initialize ROS:', error);
            this.updateConnectionStatus(false);
        }
    };

    MissionControlView.prototype.render = function() {
        this.element.innerHTML = `
            <div class="mission-control-panel">
                <div class="mission-control-header">
                    <h2>Mission Control Panel</h2>
                    <div class="connection-status">
                        <span class="status-indicator" id="ros-status">●</span>
                        <span id="connection-text">Connecting...</span>
                    </div>
                </div>

                <div class="mission-selection">
                    <label for="mission-select">Select Mission:</label>
                    <select id="mission-select" class="mission-dropdown">
                        ${this.missions.map(mission => 
                            `<option value="${mission}">${mission}</option>`
                        ).join('')}
                    </select>
                </div>

                <div class="control-buttons">
                    <button id="start-btn" class="control-button start-button">
                        <span class="button-icon">▶</span>
                        START
                    </button>
                    <button id="stop-btn" class="control-button stop-button">
                        <span class="button-icon">⏹</span>
                        STOP
                    </button>
                    <button id="reset-btn" class="control-button reset-button">
                        <span class="button-icon">↻</span>
                        RESET
                    </button>
                </div>

                <div class="status-display">
                    <div class="status-item">
                        <label>Current State:</label>
                        <span id="rover-state" class="status-value">IDLE</span>
                    </div>
                    <div class="status-item">
                        <label>Active Mission:</label>
                        <span id="active-mission" class="status-value">None</span>
                    </div>
                </div>

                <div class="message-display">
                    <div id="message-area" class="message-area"></div>
                </div>
            </div>

            <style>
                .mission-control-panel {
                    padding: 20px;
                    font-family: Arial, sans-serif;
                    max-width: 600px;
                    margin: 0 auto;
                }

                .mission-control-header {
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                    margin-bottom: 20px;
                    border-bottom: 2px solid #ddd;
                    padding-bottom: 10px;
                }

                .mission-control-header h2 {
                    margin: 0;
                    color: #333;
                }

                .connection-status {
                    display: flex;
                    align-items: center;
                    gap: 8px;
                }

                .status-indicator {
                    font-size: 20px;
                    color: #ff6b6b;
                }

                .status-indicator.connected {
                    color: #51cf66;
                }

                .mission-selection {
                    margin-bottom: 20px;
                }

                .mission-selection label {
                    display: block;
                    margin-bottom: 5px;
                    font-weight: bold;
                }

                .mission-dropdown {
                    width: 100%;
                    padding: 10px;
                    font-size: 16px;
                    border: 2px solid #ddd;
                    border-radius: 5px;
                }

                .control-buttons {
                    display: flex;
                    gap: 15px;
                    margin-bottom: 20px;
                    justify-content: center;
                }

                .control-button {
                    padding: 12px 24px;
                    font-size: 16px;
                    font-weight: bold;
                    border: none;
                    border-radius: 5px;
                    cursor: pointer;
                    transition: all 0.3s ease;
                    display: flex;
                    align-items: center;
                    gap: 8px;
                    min-width: 100px;
                    justify-content: center;
                }

                .start-button {
                    background-color: #51cf66;
                    color: white;
                }

                .start-button:hover:not(:disabled) {
                    background-color: #40c057;
                }

                .stop-button {
                    background-color: #ff6b6b;
                    color: white;
                }

                .stop-button:hover:not(:disabled) {
                    background-color: #ff5252;
                }

                .reset-button {
                    background-color: #ffd43b;
                    color: #333;
                }

                .reset-button:hover:not(:disabled) {
                    background-color: #fcc419;
                }

                .control-button:disabled {
                    opacity: 0.6;
                    cursor: not-allowed;
                }

                .control-button.processing {
                    opacity: 0.7;
                }

                .button-icon {
                    font-size: 14px;
                }

                .status-display {
                    background-color: #f8f9fa;
                    padding: 15px;
                    border-radius: 5px;
                    margin-bottom: 20px;
                }

                .status-item {
                    display: flex;
                    justify-content: space-between;
                    margin-bottom: 10px;
                }

                .status-item:last-child {
                    margin-bottom: 0;
                }

                .status-item label {
                    font-weight: bold;
                }

                .status-value {
                    padding: 4px 8px;
                    background-color: white;
                    border-radius: 3px;
                    border: 1px solid #ddd;
                }

                .message-area {
                    background-color: #f1f3f4;
                    border: 1px solid #ddd;
                    border-radius: 5px;
                    padding: 10px;
                    min-height: 100px;
                    max-height: 200px;
                    overflow-y: auto;
                    font-family: monospace;
                    font-size: 14px;
                }

                .message {
                    margin-bottom: 5px;
                    padding: 5px;
                    border-radius: 3px;
                }

                .message.success {
                    background-color: #d4edda;
                    color: #155724;
                    border: 1px solid #c3e6cb;
                }

                .message.error {
                    background-color: #f8d7da;
                    color: #721c24;
                    border: 1px solid #f5c6cb;
                }

                .message.info {
                    background-color: #d1ecf1;
                    color: #0c5460;
                    border: 1px solid #bee5eb;
                }
            </style>
        `;

        this.attachEventListeners();
        this.updateButtonStates();
    };

    MissionControlView.prototype.attachEventListeners = function() {
        const startBtn = this.element.querySelector('#start-btn');
        const stopBtn = this.element.querySelector('#stop-btn');
        const resetBtn = this.element.querySelector('#reset-btn');

        startBtn.addEventListener('click', () => this.handleStart());
        stopBtn.addEventListener('click', () => this.handleStop());
        resetBtn.addEventListener('click', () => this.handleReset());
    };

    MissionControlView.prototype.handleStart = function() {
        const missionSelect = this.element.querySelector('#mission-select');
        const selectedMission = missionSelect.value;
        
        this.callMissionService('start', selectedMission);
    };

    MissionControlView.prototype.handleStop = function() {
        this.callMissionService('stop', '');
    };

    MissionControlView.prototype.handleReset = function() {
        this.callMissionService('reset', '');
    };

    MissionControlView.prototype.callMissionService = function(requestType, missionName) {
        if (this.isProcessing || !this.missionService) {
            return;
        }

        this.isProcessing = true;
        this.updateButtonStates();
        
        const request = new ROSLIB.ServiceRequest({
            request_type: requestType,
            mission_name: missionName
        });

        this.addMessage(`Sending ${requestType.toUpperCase()} request${missionName ? ' for mission: ' + missionName : ''}...`, 'info');

        this.missionService.callService(request, (result) => {
            this.isProcessing = false;
            this.updateButtonStates();
            
            if (result.success) {
                this.addMessage(`SUCCESS: ${result.message}`, 'success');
                this.currentState = result.current_state;
                this.updateStatusDisplay();
            } else {
                this.addMessage(`ERROR: ${result.message}`, 'error');
            }
        }, (error) => {
            this.isProcessing = false;
            this.updateButtonStates();
            this.addMessage(`Service call failed: ${error}`, 'error');
            console.error('Mission service call failed:', error);
        });
    };

    MissionControlView.prototype.updateRoverStatus = function(statusMessage) {
        this.currentState = statusMessage.rover_state;
        this.activeMission = statusMessage.active_mission || 'None';
        this.updateStatusDisplay();
    };

    MissionControlView.prototype.updateStatusDisplay = function() {
        const stateElement = this.element.querySelector('#rover-state');
        const missionElement = this.element.querySelector('#active-mission');
        
        if (stateElement) {
            stateElement.textContent = this.currentState;
            stateElement.className = `status-value state-${this.currentState.toLowerCase()}`;
        }
        
        if (missionElement) {
            missionElement.textContent = this.activeMission;
        }
        
        this.updateButtonStates();
    };

    MissionControlView.prototype.updateButtonStates = function() {
        const startBtn = this.element.querySelector('#start-btn');
        const stopBtn = this.element.querySelector('#stop-btn');
        const resetBtn = this.element.querySelector('#reset-btn');
        
        if (!startBtn || !stopBtn || !resetBtn) return;

        // Reset all processing states
        [startBtn, stopBtn, resetBtn].forEach(btn => {
            btn.classList.remove('processing');
        });

        if (this.isProcessing) {
            [startBtn, stopBtn, resetBtn].forEach(btn => {
                btn.classList.add('processing');
                btn.disabled = true;
            });
            return;
        }

        // Enable/disable based on current state
        switch (this.currentState) {
            case 'IDLE':
                startBtn.disabled = false;
                stopBtn.disabled = true;
                resetBtn.disabled = false;
                break;
            case 'RUNNING':
                startBtn.disabled = true;
                stopBtn.disabled = false;
                resetBtn.disabled = false;
                break;
            case 'ERROR':
            case 'EMERGENCY_STOP':
                startBtn.disabled = true;
                stopBtn.disabled = true;
                resetBtn.disabled = false;
                break;
            case 'PAUSED':
                startBtn.disabled = false;
                stopBtn.disabled = false;
                resetBtn.disabled = false;
                break;
            default:
                startBtn.disabled = false;
                stopBtn.disabled = false;
                resetBtn.disabled = false;
        }
    };

    MissionControlView.prototype.updateConnectionStatus = function(connected) {
        const statusIndicator = this.element.querySelector('.status-indicator');
        const connectionText = this.element.querySelector('#connection-text');
        
        if (statusIndicator) {
            if (connected) {
                statusIndicator.classList.add('connected');
                statusIndicator.textContent = '●';
            } else {
                statusIndicator.classList.remove('connected');
                statusIndicator.textContent = '●';
            }
        }
        
        if (connectionText) {
            connectionText.textContent = connected ? 'Connected to ROS' : 'Disconnected';
        }
        
        this.updateButtonStates();
    };

    MissionControlView.prototype.addMessage = function(message, type = 'info') {
        const messageArea = this.element.querySelector('#message-area');
        if (!messageArea) return;

        const timestamp = new Date().toLocaleTimeString();
        const messageElement = document.createElement('div');
        messageElement.className = `message ${type}`;
        messageElement.innerHTML = `<strong>[${timestamp}]</strong> ${message}`;
        
        messageArea.appendChild(messageElement);
        messageArea.scrollTop = messageArea.scrollHeight;
        
        // Keep only last 50 messages
        const messages = messageArea.querySelectorAll('.message');
        if (messages.length > 50) {
            messageArea.removeChild(messages[0]);
        }
    };

    MissionControlView.prototype.destroy = function() {
        if (this.statusSubscriber) {
            this.statusSubscriber.unsubscribe();
        }
        if (this.ros) {
            this.ros.close();
        }
        console.log('Mission Control View destroyed');
    };

    // Expose globally
    window.MissionControlView = MissionControlView;

})()