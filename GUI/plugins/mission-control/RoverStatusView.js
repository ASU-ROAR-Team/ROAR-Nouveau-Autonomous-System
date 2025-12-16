// src/plugins/mission-control/RoverStatusView.js

(function () {
    'use strict';

    function RoverStatusView(element, openmct) {
        this.element = element;
        this.openmct = openmct;
        this.ros = null;
        this.statusSubscriber = null;
        this.cmdVelSubscriber = null; // New: Subscriber for cmd_vel
        this.roverStatus = {
            rover_state: 'UNKNOWN',
            active_mission: 'None',
            node_statuses: [],
            timestamp: 0,
            supervisor_message: 'Waiting for data...',
            // New: Add properties for linear and angular speeds
            linear_speed_x: 0.0,
            angular_speed_z: 0.0
        };
        
        this.initROS();
    }

    RoverStatusView.prototype.initROS = function() {
        try {
            // Initialize ROS connection
            this.ros = new ROSLIB.Ros({
                url: 'ws://localhost:9090'
            });

            this.ros.on('connection', () => {
                console.log('Rover Status: Connected to ROS');
                this.updateConnectionStatus(true);
            });

            this.ros.on('error', (error) => {
                console.error('Rover Status: ROS connection error:', error);
                this.updateConnectionStatus(false);
            });

            this.ros.on('close', () => {
                console.log('Rover Status: ROS connection closed');
                this.updateConnectionStatus(false);
            });

            // Subscribe to rover status
            this.statusSubscriber = new ROSLIB.Topic({
                ros: this.ros,
                name: '/rover_status',
                messageType: 'roar_msgs/RoverStatus'
            });

            this.statusSubscriber.subscribe((message) => {
                this.updateRoverStatus(message);
            });

            // New: Subscribe to /cmd_vel topic
            this.cmdVelSubscriber = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist' // Message type for Twist
            });

            this.cmdVelSubscriber.subscribe((message) => {
                this.updateCmdVelStatus(message);
            });

        } catch (error) {
            console.error('Rover Status: Failed to initialize ROS:', error);
            this.updateConnectionStatus(false);
        }
    };

    // New: Method to update cmd_vel data
    RoverStatusView.prototype.updateCmdVelStatus = function(message) {
        // Ensure linear and angular exist and have x, z properties
        this.roverStatus.linear_speed_x = message.linear ? message.linear.x : 0.0;
        this.roverStatus.angular_speed_z = message.angular ? message.angular.z : 0.0;
        // You might want to trigger a partial update if this is very frequent,
        // but for now, we'll let the main updateDisplay handle it.
        this.updateDisplay(); // Re-render to show updated speeds
    };

    RoverStatusView.prototype.render = function() {
        this.element.innerHTML = `
            <div class="rover-status-display">
                <div class="status-header">
                    <h2>Rover Status Monitor</h2>
                    <div class="connection-status">
                        <span class="status-indicator" id="ros-status">●</span>
                        <span id="connection-text">Connecting...</span>
                    </div>
                </div>

                <div class="rover-overview">
                    <div class="overview-card">
                        <h3>Rover State</h3>
                        <div class="state-display">
                            <span id="rover-state" class="rover-state unknown">UNKNOWN</span>
                        </div>
                    </div>
                    
                    <div class="overview-card">
                        <h3>Active Mission</h3>
                        <div class="mission-display">
                            <span id="active-mission">None</span>
                        </div>
                    </div>
                    
                    <div class="overview-card">
                        <h3>Last Update</h3>
                        <div class="timestamp-display">
                            <span id="last-update">Never</span>
                        </div>
                    </div>

                    <div class="overview-card">
                        <h3>Linear Speed (X)</h3>
                        <div class="speed-display">
                            <span id="linear-speed-x">0.00</span> m/s
                        </div>
                    </div>
                    <div class="overview-card">
                        <h3>Angular Speed (Z)</h3>
                        <div class="speed-display">
                            <span id="angular-speed-z">0.00</span> rad/s
                        </div>
                    </div>
                </div>

                <div class="supervisor-message">
                    <h3>Supervisor Status</h3>
                    <div id="supervisor-msg" class="supervisor-msg">Waiting for data...</div>
                </div>

                <div class="nodes-section">
                    <h3>Node Status Monitor</h3>
                    <div class="nodes-grid" id="nodes-grid">
                        </div>
                </div>
            </div>

            <style>
                .rover-status-display {
                    padding: 20px;
                    font-family: Arial, sans-serif;
                    background-color: #f8f9fa;
                    min-height: 100%;
                }

                .status-header {
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                    margin-bottom: 20px;
                    padding-bottom: 10px;
                    border-bottom: 2px solid #ddd;
                }

                .status-header h2 {
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

                .rover-overview {
                    display: grid;
                    /* Adjust grid template columns to fit 5 cards now */
                    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
                    gap: 20px;
                    margin-bottom: 30px;
                }

                .overview-card {
                    background: white;
                    padding: 20px;
                    border-radius: 8px;
                    box-shadow: 0 2px 4px rgba(0,0,0,0.1);
                    text-align: center;
                }

                .overview-card h3 {
                    margin-top: 0;
                    margin-bottom: 15px;
                    color: #495057;
                    font-size: 16px;
                }

                .state-display {
                    font-size: 24px;
                    font-weight: bold;
                }
                
                .speed-display { /* New style for speed values */
                    font-size: 20px;
                    font-weight: bold;
                    color: #007bff; /* Example color */
                }


                .rover-state {
                    padding: 8px 16px;
                    border-radius: 20px;
                    text-transform: uppercase;
                }

                .rover-state.idle {
                    background-color: #e3f2fd;
                    color: #1565c0;
                }

                .rover-state.running {
                    background-color: #e8f5e8;
                    color: #2e7d2e;
                }

                .rover-state.paused {
                    background-color: #fff3e0;
                    color: #f57c00;
                }

                .rover-state.error {
                    background-color: #ffebee;
                    color: #c62828;
                }

                .rover-state.emergency_stop {
                    background-color: #ffcdd2;
                    color: #b71c1c;
                }

                .rover-state.unknown {
                    background-color: #f5f5f5;
                    color: #757575;
                }

                .mission-display, .timestamp-display {
                    font-size: 18px;
                    font-weight: 500;
                    color: #333;
                }

                .supervisor-message {
                    background: white;
                    padding: 20px;
                    border-radius: 8px;
                    box-shadow: 0 2px 4px rgba(0,0,0,0.1);
                    margin-bottom: 30px;
                }

                .supervisor-message h3 {
                    margin-top: 0;
                    margin-bottom: 10px;
                    color: #495057;
                }

                .supervisor-msg {
                    padding: 10px;
                    background-color: #f8f9fa;
                    border-left: 4px solid #007bff;
                    border-radius: 4px;
                    font-family: monospace;
                }

                .nodes-section {
                    background: white;
                    padding: 20px;
                    border-radius: 8px;
                    box-shadow: 0 2px 4px rgba(0,0,0,0.1);
                }

                .nodes-section h3 {
                    margin-top: 0;
                    margin-bottom: 20px;
                    color: #495057;
                }

                .nodes-grid {
                    display: grid;
                    grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
                    gap: 15px;
                }

                .node-card {
                    border: 1px solid #dee2e6;
                    border-radius: 6px;
                    padding: 15px;
                    background-color: #ffffff;
                }

                .node-card.running {
                    border-left: 4px solid #28a745;
                }

                .node-card.stopped {
                    border-left: 4px solid #ffc107;
                }

                .node-card.error {
                    border-left: 4px solid #dc3545;
                }

                .node-card.unknown, .node-card.inactive, .node-card.undefined { /* Added inactive and undefined */
                    border-left: 4px solid #6c757d;
                }

                .node-card.healthy { /* Assuming you might use HEALTHY status */
                    border-left: 4px solid #28a745;
                }
                .node-card.warning { /* Assuming you might use WARNING status */
                    border-left: 4px solid #ffc107;
                }
                .node-card.failed { /* Assuming you might use FAILED status */
                    border-left: 4px solid #dc3545;
                }


                .node-header {
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                    margin-bottom: 10px;
                }

                .node-name {
                    font-weight: bold;
                    font-size: 16px;
                }

                .node-status {
                    padding: 4px 8px;
                    border-radius: 12px;
                    font-size: 12px;
                    font-weight: bold;
                    text-transform: uppercase;
                }

                .node-status.running, .node-status.healthy {
                    background-color: #d4edda;
                    color: #155724;
                }

                .node-status.stopped, .node-status.inactive {
                    background-color: #fff3cd;
                    color: #856404;
                }

                .node-status.error, .node-status.failed {
                    background-color: #f8d7da;
                    color: #721c24;
                }

                .node-status.unknown, .node-status.undefined, .node-status.warning {
                    background-color: #e2e3e5;
                    color: #383d41;
                }

                .node-details {
                    display: grid;
                    grid-template-columns: 1fr 1fr;
                    gap: 10px;
                    font-size: 14px;
                }

                .detail-item {
                    display: flex;
                    justify-content: space-between;
                }

                .detail-label {
                    color: #6c757d;
                }

                .detail-value {
                    font-weight: 500;
                }

                .node-error {
                    margin-top: 10px;
                    padding: 8px;
                    background-color: #f8d7da;
                    color: #721c24;
                    border-radius: 4px;
                    font-size: 12px;
                    font-family: monospace;
                }

                .no-nodes {
                    text-align: center;
                    color: #6c757d;
                    font-style: italic;
                    padding: 40px;
                }
            </style>
        `;

        this.updateDisplay();
    };

    RoverStatusView.prototype.updateRoverStatus = function(statusMessage) {
        this.roverStatus.rover_state = statusMessage.rover_state;
        this.roverStatus.active_mission = statusMessage.active_mission;
        this.roverStatus.node_statuses = statusMessage.node_statuses;
        this.roverStatus.timestamp = statusMessage.timestamp;
        this.roverStatus.supervisor_message = statusMessage.supervisor_message;
        // Note: linear_speed_x and angular_speed_z are updated by updateCmdVelStatus
        this.updateDisplay();
    };

    RoverStatusView.prototype.updateDisplay = function() {
        this.updateRoverState();
        this.updateMissionInfo();
        this.updateTimestamp();
        this.updateSupervisorMessage();
        this.updateSpeedDisplays(); // New: Call to update speeds
        this.updateNodesDisplay();
    };

    // ... (rest of the updateRoverState, updateMissionInfo, updateTimestamp, updateSupervisorMessage methods remain the same) ...

    RoverStatusView.prototype.updateRoverState = function() {
        const stateElement = this.element.querySelector('#rover-state');
        if (stateElement) {
            const state = this.roverStatus.rover_state || 'UNKNOWN';
            stateElement.textContent = state;
            stateElement.className = `rover-state ${state.toLowerCase().replace(/_/g, '')}`; // Fixed regex for all underscores
        }
    };

    RoverStatusView.prototype.updateMissionInfo = function() {
        const missionElement = this.element.querySelector('#active-mission');
        if (missionElement) {
            missionElement.textContent = this.roverStatus.active_mission || 'None';
        }
    };

    RoverStatusView.prototype.updateTimestamp = function() {
        const timestampElement = this.element.querySelector('#last-update');
        if (timestampElement) {
            if (this.roverStatus.timestamp && this.roverStatus.timestamp > 0) {
                const date = new Date(this.roverStatus.timestamp * 1000);
                timestampElement.textContent = date.toLocaleTimeString();
            } else {
                timestampElement.textContent = 'Never';
            }
        }
    };

    RoverStatusView.prototype.updateSupervisorMessage = function() {
        const msgElement = this.element.querySelector('#supervisor-msg');
        if (msgElement) {
            msgElement.textContent = this.roverStatus.supervisor_message || 'No message';
        }
    };

    // New: Method to update speed display elements
    RoverStatusView.prototype.updateSpeedDisplays = function() {
        const linearSpeedElement = this.element.querySelector('#linear-speed-x');
        const angularSpeedElement = this.element.querySelector('#angular-speed-z');

        if (linearSpeedElement) {
            linearSpeedElement.textContent = this.roverStatus.linear_speed_x.toFixed(2);
        }
        if (angularSpeedElement) {
            angularSpeedElement.textContent = this.roverStatus.angular_speed_z.toFixed(2);
        }
    };

    RoverStatusView.prototype.updateNodesDisplay = function() {
        const nodesGrid = this.element.querySelector('#nodes-grid');
        if (!nodesGrid) return;

        const nodeStatuses = this.roverStatus.node_statuses || [];
        
        if (nodeStatuses.length === 0) {
            nodesGrid.innerHTML = '<div class="no-nodes">No node data available</div>';
            return;
        }

        nodesGrid.innerHTML = nodeStatuses.map(node => {
            // Ensure statusClass can handle "HEALTHY", "WARNING", "FAILED" from Python side
            const statusClass = node.status ? node.status.toLowerCase().replace(/_/g, '') : 'unknown'; // Ensure consistent class names
            const cpuUsage = node.cpu_usage ? node.cpu_usage.toFixed(1) : '0.0';
            const memoryUsage = node.memory_usage ? node.memory_usage.toFixed(1) : '0.0';
            const pid = node.pid && node.pid > 0 ? node.pid : 'N/A';
            
            return `
                <div class="node-card ${statusClass}">
                    <div class="node-header">
                        <span class="node-name">${node.node_name || 'Unknown'}</span>
                        <span class="node-status ${statusClass}">${node.status || 'UNKNOWN'}</span>
                    </div>
                    <div class="node-details">
                        <div class="detail-item">
                            <span class="detail-label">CPU:</span>
                            <span class="detail-value">${cpuUsage}%</span>
                        </div>
                        <div class="detail-item">
                            <span class="detail-label">Memory:</span>
                            <span class="detail-value">${memoryUsage} MB</span>
                        </div>
                        <div class="detail-item">
                            <span class="detail-label">PID:</span>
                            <span class="detail-value">${pid}</span>
                        </div>
                    </div>
                    ${node.last_error ? `<div class="node-error">Error: ${node.last_error}</div>` : ''}
                </div>
            `;
        }).join('');
    };

    RoverStatusView.prototype.updateConnectionStatus = function(connected) {
        const statusIndicator = this.element.querySelector('.status-indicator');
        const connectionText = this.element.querySelector('#connection-text');
        
        if (statusIndicator) {
            if (connected) {
                statusIndicator.classList.add('connected');
            } else {
                statusIndicator.classList.remove('connected');
            }
        }
        
        if (connectionText) {
            connectionText.textContent = connected ? 'Connected to ROS' : 'Disconnected';
        }
    };

    RoverStatusView.prototype.destroy = function() {
        if (this.statusSubscriber) {
            this.statusSubscriber.unsubscribe();
        }
        if (this.cmdVelSubscriber) { // New: Unsubscribe from cmd_vel
            this.cmdVelSubscriber.unsubscribe();
        }
        if (this.ros) {
            this.ros.close();
        }
        console.log('Rover Status View destroyed');
    };

    // Expose globally
    window.RoverStatusView = RoverStatusView;

})();