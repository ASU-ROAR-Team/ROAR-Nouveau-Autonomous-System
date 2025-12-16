// src/plugins/Drilling-Control/DrillingControlView.js

(function () {
    class DrillingControlView {
        constructor(element, openmct) {
            this.element = element;
            this.openmct = openmct;
            console.log("DrillingControlView constructor: this.openmct is", this.openmct); 

            this.ros = null;
            this.rosConnected = false;
            this.drillingCommandPublisher = null; 
            this.drillingStatusSubscriber = null;
            this.fsmStateSubscriber = null;
            this.roverStatusSubscriber = null;
            this.currentRoverState = { rover_state: 'IDLE', active_mission: '' };
            this.webcamSubscriber = null;

            this.rosStatusDot = null;
            this.rosStatus = null;
            this.fsmStateDisplay = null;
            this.platformDepthDisplay = null;
            this.sampleWeightDisplay = null;

            this.platformUpButton = null;
            this.platformDownButton = null;
            this.augerToggleSwitch = null; 
            this.gateToggleSwitch = null; 

            this.last_known_height = 0.0;
            
            this.currentManualInputState = {
                manual_up: false,
                manual_down: false,
                auger_on: false,
                gate_open: false
            };

            this.webcamImageElement = null; 
            this.webcamStatusMessageElement = null;
            this.webcamSnapshotButton = null;
            this.webcamInnerSnapshotCircle = null;
        }

        handleRosConnection = () => {
            console.log('Connected to ROS websocket server.');
            this.rosConnected = true;
            if (this.rosStatus) {
                this.rosStatus.textContent = 'Connected to ROS';
                this.rosStatus.classList.remove('error');
                this.rosStatus.classList.add('connected');
            }
            if (this.rosStatusDot) {
                this.rosStatusDot.classList.remove('error');
                this.rosStatusDot.classList.add('connected');
            }
            this.startRosWebcam();
        };

        handleRosError = (error) => {
            console.error('ROS connection error:', error);
            this.rosConnected = false;
            if (this.rosStatus) {
                this.rosStatus.textContent = 'ROS Connection Error!';
                this.rosStatus.classList.remove('connected');
                this.rosStatus.classList.add('error');
            }
            if (this.rosStatusDot) {
                this.rosStatusDot.classList.remove('connected');
                this.rosStatusDot.classList.add('error');
            }
            this.stopRosWebcam();
        };

        handleRosClose = () => {
            console.warn('ROS connection closed. Attempting to reconnect...');
            this.rosConnected = false;
            if (this.rosStatus) {
                this.rosStatus.textContent = 'Disconnected from ROS';
                this.rosStatus.classList.remove('connected');
                this.rosStatus.classList.add('error');
            }
            if (this.rosStatusDot) {
                this.rosStatusDot.classList.remove('connected');
                this.rosStatusDot.classList.add('error');
            }
            this.stopRosWebcam();
            setTimeout(() => {
                console.log('Attempting to reconnect to ROS...');
                this.connectToROS();
            }, 3000);
        };

        handleEditModeChange = (isEditing) => {
            if (isEditing) {
                this.stopRosWebcam();
                this.displayWebcamStatus('Webcam: In edit mode. Stream paused.', 'info');
            } else {
                this.startRosWebcam();
            }
        };
        
        displayWebcamStatus = (message, type = 'info') => {
            if (this.webcamStatusMessageElement) {
                this.webcamStatusMessageElement.textContent = message;
                this.webcamStatusMessageElement.classList.remove('info', 'error', 'warning');
                this.webcamStatusMessageElement.classList.add(type);
                this.webcamStatusMessageElement.style.display = 'block';
            }
        };

        hideWebcamStatus = () => {
            if (this.webcamStatusMessageElement) {
                this.webcamStatusMessageElement.textContent = '';
                this.webcamStatusMessageElement.classList.remove('info', 'error', 'warning');
                this.webcamStatusMessageElement.style.display = 'none';
            }
        };
        
        startRosWebcam = () => {
            if (!this.rosConnected) {
                this.displayWebcamStatus('Waiting for ROS connection...', 'info');
                return;
            }
            if (!this.webcamImageElement) {
                this.displayWebcamStatus('Webcam display element not found.', 'error');
                return;
            }
            if (this.webcamSubscriber) {
                console.log('Webcam subscriber already active.');
                return;
            }
            
            this.displayWebcamStatus('Connecting to ROS camera topic...', 'info');
            
            this.webcamSubscriber = new window.ROSLIB.Topic({
                ros: this.ros,
                name: '/logitech_1/image_raw/compressed', 
                messageType: 'sensor_msgs/CompressedImage'
            });

            this.webcamSubscriber.subscribe(this.handleImageMessage.bind(this));
            console.log('Subscribed to ROS camera topic.');
        };

        handleImageMessage = (message) => {
            const imageUri = `data:image/jpeg;base64,${message.data}`;
            if (this.webcamImageElement) {
                this.webcamImageElement.src = imageUri;
                this.webcamImageElement.style.display = 'block';
            }
            this.hideWebcamStatus();
            if (this.webcamSnapshotButton) {
                this.webcamSnapshotButton.style.display = 'flex';
            }
        };

        stopRosWebcam = () => {
            if (this.webcamSubscriber) {
                this.webcamSubscriber.unsubscribe();
                this.webcamSubscriber = null;
                console.log('Unsubscribed from ROS camera topic.');
            }
            if (this.webcamImageElement) {
                this.webcamImageElement.src = '';
                this.webcamImageElement.style.display = 'none';
            }
            if (this.webcamSnapshotButton) {
                this.webcamSnapshotButton.style.display = 'none';
            }
            this.displayWebcamStatus('Webcam stream paused/stopped.', 'info');
        };

        takeWebcamSnapshot = () => {
            if (!this.webcamImageElement || !this.webcamImageElement.src) {
                console.warn('Webcam image not found or not loaded.');
                if (this.openmct && this.openmct.notifications) {
                    this.openmct.notifications.warn('Webcam feed not found.');
                }
                return;
            }
            
            const link = document.createElement('a');
            link.href = this.webcamImageElement.src;
            link.download = 'webcam_snapshot.jpg';
            link.click();

            if (this.openmct && this.openmct.notifications) {
                this.openmct.notifications.info('Webcam snapshot taken!');
            }
        };


        handleSnapshotButtonMouseDown = () => {
            if (this.webcamSnapshotButton) {
                this.webcamSnapshotButton.style.transform = 'translateX(-50%) scale(0.95)';
                this.webcamSnapshotButton.style.boxShadow = '0 2px 4px rgba(0,0,0,0.2)';
            }
        };

        handleSnapshotButtonMouseUp = () => {
            if (this.webcamSnapshotButton) {
                this.webcamSnapshotButton.style.transform = 'translateX(-50%) scale(1)';
                this.webcamSnapshotButton.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)';
            }
        };

        handleSnapshotButtonMouseLeave = () => {
            if (this.webcamSnapshotButton) {
                this.webcamSnapshotButton.style.transform = 'translateX(-50%) scale(1)';
                this.webcamSnapshotButton.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)';
            }
        };

        render() {
            fetch('./plugins/Drilling-Control/DrillingControlView.html')
                .then(response => {
                    if (!response.ok) {
                        throw new Error(`HTTP error! status: ${response.status}`);
                    }
                    return response.text();
                })
                .then(html => {
                    this.element.innerHTML = html;
                    const link = document.createElement('link');
                    link.rel = 'stylesheet';
                    link.href = './plugins/Drilling-Control/DrillingControlView.css';
                    document.head.appendChild(link);

                    this.initializeUI();
                    this.connectToROS();
                    this.openmct.editor.on('isEditing', this.handleEditModeChange); 
                })
                .catch(error => {
                    console.error('Error loading DrillingControlView.html:', error);
                    this.element.innerHTML = `<p style="color: red;">Error loading drilling control UI.</p>`;
                });
        }

        initializeUI() {
            this.rosStatusDot = this.element.querySelector('#drillingRosStatusDot');
            this.rosStatus = this.element.querySelector('#drillingRosStatus');
            this.fsmStateDisplay = this.element.querySelector('#drillingFsmState');
            this.platformDepthDisplay = this.element.querySelector('#drillingPlatformDepth');
            this.sampleWeightDisplay = this.element.querySelector('#drillingSampleWeight');

            this.platformUpButton = this.element.querySelector('#drillingPlatformUpButton');
            this.platformDownButton = this.element.querySelector('#drillingPlatformDownButton');
            this.augerToggleSwitch = this.element.querySelector('#drillingAugerToggle');
            this.gateToggleSwitch = this.element.querySelector('#drillingGateToggle');

            const webcamContainer = this.element.querySelector('#drillingWebcamContainer');
            if (webcamContainer) {
                this.webcamImageElement = webcamContainer.querySelector('#drillingWebcamImage');
                this.webcamSnapshotButton = webcamContainer.querySelector('#drillingSnapshotButton');
                this.webcamInnerSnapshotCircle = this.webcamSnapshotButton.querySelector('.drilling-snapshot-inner-circle');
                this.webcamStatusMessageElement = webcamContainer.querySelector('#drillingWebcamStatusMessage');

                if (this.webcamImageElement) {
                    this.webcamImageElement.style.display = 'none';
                }
                if (this.webcamSnapshotButton) {
                    this.webcamSnapshotButton.style.display = 'none';
                }
                if (this.webcamStatusMessageElement) {
                    this.webcamStatusMessageElement.style.display = 'block';
                    this.displayWebcamStatus('Loading webcam...', 'info');
                }
            } else {
                console.error("DrillingControlView: #drillingWebcamContainer not found in HTML. Webcam functionality will not work.");
                const webcamAreaDiv = this.element.querySelector('.drilling-webcam-feed-wrapper');
                if (webcamAreaDiv) {
                    webcamAreaDiv.innerHTML = `<p style="color: red; text-align: center; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);">Error: Webcam container not found in HTML. Please ensure 'drillingWebcamContainer' ID exists.</p>`;
                }
            }

            this.addEventListeners();
            this.updateManualControlUIState();
        }

        addEventListeners() {
            const addMomentaryListener = (button, command, value) => {
                if (button) {
                    button.addEventListener('mousedown', () => { if (!button.disabled) this.handleManualButtonClick(command, value); });
                    button.addEventListener('mouseup', () => { if (!button.disabled) this.handleManualButtonClick(command, false); });
                    button.addEventListener('mouseleave', () => { if (!button.disabled) this.handleManualButtonClick(command, false); });
                }
            };
            addMomentaryListener(this.platformUpButton, 'manual_up', true);
            addMomentaryListener(this.platformDownButton, 'manual_down', true);

            if (this.augerToggleSwitch) {
                this.augerToggleSwitch.addEventListener('change', () => {
                    this.handleSwitchChange('auger_on', this.augerToggleSwitch.checked);
                });
            }
            if (this.gateToggleSwitch) {
                this.gateToggleSwitch.addEventListener('change', () => {
                    this.handleSwitchChange('gate_open', this.gateToggleSwitch.checked);
                });
            }

            if (this.webcamSnapshotButton) {
                this.webcamSnapshotButton.addEventListener('click', this.takeWebcamSnapshot);
                this.webcamSnapshotButton.addEventListener('mousedown', this.handleSnapshotButtonMouseDown);
                this.webcamSnapshotButton.addEventListener('mouseup', this.handleSnapshotButtonMouseUp);
                this.webcamSnapshotButton.addEventListener('mouseleave', this.handleSnapshotButtonMouseLeave);
            }
        }
        
        handleSwitchChange(commandType, value) {
            if (this.currentRoverState.active_mission.toLowerCase() !== 'teleoperation') {
                this.augerToggleSwitch.checked = this.currentManualInputState.auger_on;
                this.gateToggleSwitch.checked = this.currentManualInputState.gate_open;
                if (this.openmct && this.openmct.notifications) {
                    this.openmct.notifications.warn('Manual controls are disabled outside of Teleoperation mode.');
                }
                return;
            }
            this.currentManualInputState[commandType] = value;
            this.publishDrillingCommand();
        }

        publishDrillingCommand() {
            if (this.currentRoverState.active_mission.toLowerCase() !== 'teleoperation') {
                console.warn('Not in Teleoperation mode. Command not sent.');
                if (this.openmct && this.openmct.notifications) {
                    this.openmct.notifications.warn('Manual controls are disabled outside of Teleoperation mode.');
                }
                return;
            }

            if (!this.rosConnected || !this.drillingCommandPublisher) {
                console.warn('ROS is not connected or command publisher is not initialized. Command not sent.');
                return;
            }

            if (typeof ROSLIB === 'undefined' || typeof ROSLIB.Message === 'undefined') {
                console.error('ROSLIB or ROSLIB.Message is not defined. Cannot send command.');
                return;
            }
            
            let target_height = 0.0;
            if (this.currentManualInputState.manual_up || this.currentManualInputState.manual_down) {
                target_height = 0.0; 
            } else {
                target_height = this.last_known_height;
            }

            const drillingCommandMsg = new ROSLIB.Message({
                target_height_cm: target_height,
                manual_up: this.currentManualInputState.manual_up,
                manual_down: this.currentManualInputState.manual_down,
                auger_on: this.currentManualInputState.auger_on,
                gate_open: this.currentManualInputState.gate_open
            });

            this.drillingCommandPublisher.publish(drillingCommandMsg);
            console.log("Published Drilling Command:", drillingCommandMsg);
        }

        handleManualButtonClick(commandType, value) {
            if (this.currentRoverState.active_mission.toLowerCase() !== 'teleoperation') {
                return;
            }

            this.currentManualInputState[commandType] = value;
            
            if (commandType === 'manual_up' && value) {
                this.currentManualInputState.manual_down = false;
            } else if (commandType === 'manual_down' && value) {
                this.currentManualInputState.manual_up = false;
            }
            
            this.publishDrillingCommand();
        }

        connectToROS() {
            if (typeof window.ROSLIB === 'undefined') {
                console.error("ROSLIB is not defined. Ensure ros-lib.js is loaded.");
                this.updateRosStatus(false, 'ROSLIB not loaded');
                return;
            }

            this.ros = new window.ROSLIB.Ros({ url: 'ws://localhost:9090' });
            this.ros.on('connection', this.handleRosConnection);
            this.ros.on('error', this.handleRosError);
            this.ros.on('close', this.handleRosClose);

            this.drillingCommandPublisher = new window.ROSLIB.Topic({
                ros: this.ros,
                name: '/drilling/command_to_actuators',
                messageType: 'roar_msgs/DrillingCommand'
            });

            this.drillingStatusSubscriber = new window.ROSLIB.Topic({
                ros: this.ros,
                name: '/drilling/feedback',
                messageType: 'roar_msgs/DrillingStatus'
            });
            this.drillingStatusSubscriber.subscribe(this.handleDrillingStatus);
            
            this.fsmStateSubscriber = new window.ROSLIB.Topic({
                ros: this.ros,
                name: '/drilling_fsm_state',
                messageType: 'std_msgs/String'
            });
            this.fsmStateSubscriber.subscribe(this.handleFsmState);

            this.roverStatusSubscriber = new window.ROSLIB.Topic({
                ros: this.ros,
                name: '/rover_status',
                messageType: 'roar_msgs/RoverStatus'
            });
            this.roverStatusSubscriber.subscribe(this.handleRoverStatus);
        }

        handleDrillingStatus = (message) => {
            const positiveHeight =(message.current_height);
            this.last_known_height = positiveHeight;
            if (this.platformDepthDisplay) {
                this.platformDepthDisplay.textContent = positiveHeight.toFixed(1);
            }
            if (this.sampleWeightDisplay) {
                this.sampleWeightDisplay.textContent = message.current_weight.toFixed(0);
            }
        }
        
        handleFsmState = (message) => {
            if (this.fsmStateDisplay) {
                this.fsmStateDisplay.textContent = message.data;
            }
        }

        handleRoverStatus = (message) => {
            this.currentRoverState = {
                rover_state: message.rover_state,
                active_mission: message.active_mission
            };
            console.log("Rover Status Received:", this.currentRoverState);
            this.updateManualControlUIState();
        }

        updateManualControlUIState = () => {
            const buttons = [
                this.platformUpButton, this.platformDownButton,
            ];
            const switches = [
                this.augerToggleSwitch, this.gateToggleSwitch
            ];
            const enableManualControls = this.currentRoverState.active_mission.toLowerCase() === 'teleoperation';

            buttons.forEach(button => {
                if (button) {
                    button.disabled = !enableManualControls;
                    if (enableManualControls) {
                        button.classList.remove('disabled-manual-control');
                    } else {
                        button.classList.add('disabled-manual-control');
                    }
                }
            });
            
            switches.forEach(sw => {
                if (sw) {
                    sw.disabled = !enableManualControls;
                    const parentContainer = sw.closest('.drilling-switch-container');
                    if (parentContainer) {
                         if (enableManualControls) {
                            parentContainer.classList.remove('disabled');
                        } else {
                            parentContainer.classList.add('disabled');
                        }
                    }
                }
            });

            const manualControlMessage = this.element.querySelector('.drilling-control-section p');
            if (manualControlMessage) {
                if (enableManualControls) {
                    manualControlMessage.textContent = 'These controls directly command the rig. They are active when the system is in "IDLE" state.';
                    manualControlMessage.style.color = '#666';
                } else {
                    manualControlMessage.textContent = `Manual controls are disabled outside of 'Teleoperation' mission. Current mission: ${this.currentRoverState.active_mission || 'None'}.`;
                    manualControlMessage.style.color = '#e74c3c';
                }
            }
        }

        updateRosStatus(isConnected, message = '') {
            if (isConnected) {
                this.rosConnected = true;
                if (this.rosStatusDot) {
                    this.rosStatusDot.classList.remove('error');
                    this.rosStatusDot.classList.add('connected');
                }
                if (this.rosStatus) {
                    this.rosStatus.textContent = 'Connected to ROS';
                    this.rosStatus.classList.remove('error');
                    this.rosStatus.classList.add('connected');
                }
            } else {
                this.rosConnected = false;
                if (this.rosStatusDot) {
                    this.rosStatusDot.classList.remove('connected');
                    this.rosStatusDot.classList.add('error');
                }
                if (this.rosStatus) {
                    this.rosStatus.textContent = message || 'Disconnected';
                    this.rosStatus.classList.remove('connected');
                    this.rosStatus.classList.add('error');
                }
            }
        }
        
        destroy() {
            console.log('Destroying DrillingControlView...');
            
            if (this.webcamSnapshotButton) {
                this.webcamSnapshotButton.removeEventListener('click', this.takeWebcamSnapshot);
                this.webcamSnapshotButton.removeEventListener('mousedown', this.handleSnapshotButtonMouseDown);
                this.webcamSnapshotButton.removeEventListener('mouseup', this.handleSnapshotButtonMouseUp);
                this.webcamSnapshotButton.removeEventListener('mouseleave', this.handleSnapshotButtonMouseLeave);
            }

            this.stopRosWebcam();

            if (this.drillingCommandPublisher) {
                this.drillingCommandPublisher.unadvertise();
            }
            if (this.drillingStatusSubscriber) {
                this.drillingStatusSubscriber.unsubscribe();
            }
            if (this.fsmStateSubscriber) {
                this.fsmStateSubscriber.unsubscribe();
            }
            if (this.roverStatusSubscriber) {
                this.roverStatusSubscriber.unsubscribe();
            }
            
            if (this.webcamSubscriber) {
                this.webcamSubscriber.unsubscribe();
            }

            if (this.ros && this.ros.isConnected) {
                this.ros.off('connection', this.handleRosConnection);
                this.ros.off('error', this.handleRosError);
                this.ros.off('close', this.handleRosClose);
                this.ros.close();
            }

            if (this.openmct && this.openmct.editor) {
                this.openmct.editor.off('isEditing', this.handleEditModeChange);
            }
            
            this.platformUpButton = null;
            this.platformDownButton = null;
            this.augerToggleSwitch = null;
            this.gateToggleSwitch = null;
            this.webcamImageElement = null;
            this.webcamStatusMessageElement = null;
            this.webcamSnapshotButton = null;
            this.webcamInnerSnapshotCircle = null;
            this.rosStatusDot = null;
            this.rosStatus = null;
            this.fsmStateDisplay = null;
            this.platformDepthDisplay = null;
            this.sampleWeightDisplay = null;
            this.ros = null;
            this.openmct = null;
            this.currentRoverState = null;
            this.roverStatusSubscriber = null;
            this.webcamSubscriber = null;

            this.element.innerHTML = '';
        }
    }

    window.DrillingControlView = DrillingControlView;

})();
