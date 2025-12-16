// src/plugins/joystick-control/JoystickView.js

// IMPORTANT: Ensure roslib.min.js is loaded BEFORE this script in index.html
// e.g., <script src="https://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>

// Wrap in an IIFE to keep local variables private, but expose the main class
(function () {
    class JoystickView {
        constructor(element, openmct) {
            this.element = element;
            this.openmct = openmct;
            this.htmlContent = null; 

            this.canvas = null;
            this.ctx = null;
            this.joystickRadius = 0;
            this.thumbRadius = 20; // Radius of the draggable thumb
            this.joystickCenterX = 0; // Center X of the joystick base
            this.joystickCenterY = 0; // Center Y of the joystick base
            this.thumbX = 0; // Current X position of the thumb
            this.thumbY = 0; // Current Y position of the thumb
            this.isDragging = false; // Flag to track if the thumb is being dragged

            this.linearSpeedSlider = null;
            this.angularSpeedSlider = null;
            this.linearSpeedValueSpan = null;
            this.angularSpeedValueSpan = null;
            this.joystickStatus = null; // Element to display ROS connection status

            this.ros = null;
            this.joyTopicPublisher = null;
            this.rosConnected = false;

            // Rover status properties to control joystick activation
            this.roverStatusSubscriber = null;
            this.currentRoverState = { rover_state: 'IDLE', active_mission: '' }; // Initialize with default state

            // Bind all event handlers to the class instance to maintain 'this' context
            this.onMouseDown = this.onMouseDown.bind(this);
            this.onMouseMove = this.onMouseMove.bind(this);
            this.onMouseUp = this.onMouseUp.bind(this);
            // No onMouseLeave handler to prevent thumb snapping back when cursor leaves canvas
            this.updateSpeedValues = this.updateSpeedValues.bind(this);
            this.handleRosConnection = this.handleRosConnection.bind(this);
            this.handleRosError = this.handleRosError.bind(this);
            this.handleRosClose = this.handleRosClose.bind(this);
            this.handleRoverStatus = this.handleRoverStatus.bind(this); 
            this.updateJoystickUIState = this.updateJoystickUIState.bind(this); 
        }

        // This method is called by OpenMCT to render the view
        render() {
            // Load the HTML content from the specified path
            fetch('./plugins/joystick-control/JoystickView.html')
                .then(response => response.text())
                .then(html => {
                    this.htmlContent = html; // Store HTML content (optional, not strictly used after render)
                    this.element.innerHTML = html; // Insert HTML into the provided element
                    this.initializeUI(); // Initialize UI elements and canvas
                    this.connectToROS(); // Establish ROS connection
                })
                .catch(error => {
                    console.error('Error loading JoystickView.html:', error);
                    this.element.innerHTML = '<p style="color: red;">Error loading joystick UI.</p>';
                });
        }

        initializeUI() {
            // Get references to HTML elements
            this.canvas = this.element.querySelector('#joystickCanvas');
            this.ctx = this.canvas.getContext('2d'); // Get 2D rendering context
            this.linearSpeedSlider = this.element.querySelector('#linearSpeed');
            this.angularSpeedSlider = this.element.querySelector('#angularSpeed');
            this.linearSpeedValueSpan = this.element.querySelector('#linearSpeedValue');
            this.angularSpeedValueSpan = this.element.querySelector('#angularSpeedValue');
            this.joystickStatus = this.element.querySelector('#joystickStatus');
            this.joystickControlMessage = this.element.querySelector('#joystickControlMessage');

            // Set canvas dimensions based on its parent's computed style
            // This ensures the canvas scales with its container
            const wrapper = this.canvas.parentElement;
            this.canvas.width = wrapper.clientWidth;
            this.canvas.height = wrapper.clientHeight;

            // Calculate joystick properties based on canvas size
            this.joystickRadius = Math.min(this.canvas.width, this.canvas.height) / 2 - 10; // 10px padding
            this.joystickCenterX = this.canvas.width / 2;
            this.joystickCenterY = this.canvas.height / 2;
            this.thumbX = this.joystickCenterX; // Initial thumb position (center)
            this.thumbY = this.joystickCenterY; // Initial thumb position (center)

            this.drawJoystick(); // Draw the joystick for the first time
            this.addEventListeners(); // Attach event listeners
            this.updateSpeedValues(); // Initialize slider display
            this.updateJoystickUIState(); // Set initial joystick UI state based on default rover state
        }

        addEventListeners() {
            // Mouse events for joystick interaction on the canvas
            this.canvas.addEventListener('mousedown', this.onMouseDown);
            // Listen for mouseup and mousemove on the *document* to capture events even if cursor leaves canvas
            document.addEventListener('mousemove', this.onMouseMove);
            document.addEventListener('mouseup', this.onMouseUp);

            // Touch events for mobile interaction
            this.canvas.addEventListener('touchstart', (e) => {
                e.preventDefault(); // Prevent default touch actions like scrolling
                this.onMouseDown(e.touches[0]); // Use the first touch point
            });
            document.addEventListener('touchmove', (e) => { // Document-level touchmove for continuous drag
                e.preventDefault();
                this.onMouseMove(e.touches[0]);
            });
            document.addEventListener('touchend', this.onMouseUp);
            document.addEventListener('touchcancel', this.onMouseUp);

            // Event listeners for speed sliders
            this.linearSpeedSlider.addEventListener('input', this.updateSpeedValues);
            this.angularSpeedSlider.addEventListener('input', this.updateSpeedValues);

            // Handle window resize to redraw canvas and adjust layout
            this._resizeListener = () => { // Store as a property to be able to remove it later
                if (this.canvas) {
                    const wrapper = this.canvas.parentElement;
                    this.canvas.width = wrapper.clientWidth;
                    this.canvas.height = wrapper.clientHeight;
                    this.joystickRadius = Math.min(this.canvas.width, this.canvas.height) / 2 - 10;
                    this.joystickCenterX = this.canvas.width / 2;
                    this.joystickCenterY = this.canvas.height / 2;
                    // Reset thumb position on resize to center
                    this.thumbX = this.joystickCenterX;
                    this.thumbY = this.joystickCenterY;
                    this.drawJoystick(); // Redraw joystick with new dimensions
                    this.publishJoy(0, 0); // Send zero input on resize to stop movement
                }
            };
            window.addEventListener('resize', this._resizeListener);
        }

        removeEventListeners() {
            // Remove all attached event listeners to prevent memory leaks
            if (this.canvas) {
                this.canvas.removeEventListener('mousedown', this.onMouseDown);
                this.canvas.removeEventListener('touchstart', (e) => { e.preventDefault(); this.onMouseDown(e.touches[0]); });
            }
            // These are document-level, so remove them from the document
            document.removeEventListener('mousemove', this.onMouseMove);
            document.removeEventListener('mouseup', this.onMouseUp);
            document.removeEventListener('touchmove', (e) => { e.preventDefault(); this.onMouseMove(e.touches[0]); });
            document.removeEventListener('touchend', this.onMouseUp);
            document.removeEventListener('touchcancel', this.onMouseUp);
            
            if (this.linearSpeedSlider) {
                this.linearSpeedSlider.removeEventListener('input', this.updateSpeedValues);
            }
            if (this.angularSpeedSlider) {
                this.angularSpeedSlider.removeEventListener('input', this.updateSpeedValues);
            }
            if (this._resizeListener) {
                window.removeEventListener('resize', this._resizeListener);
            }
        }

        updateSpeedValues() {
            // Update the displayed numeric values for linear and angular speed
            const currentLinear = parseFloat(this.linearSpeedSlider.value);
            const currentAngular = parseFloat(this.angularSpeedSlider.value);
            this.linearSpeedValueSpan.textContent = currentLinear.toFixed(1);
            this.angularSpeedValueSpan.textContent = currentAngular.toFixed(1);
        }

        drawJoystick() {
            if (!this.ctx) return; // Ensure context is available

            this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height); // Clear the entire canvas

            // Draw outer circle (joystick base)
            this.ctx.beginPath();
            this.ctx.arc(this.joystickCenterX, this.joystickCenterY, this.joystickRadius, 0, Math.PI * 2, false);
            this.ctx.strokeStyle = '#666'; // Stroke color
            this.ctx.lineWidth = 3; // Line width
            this.ctx.stroke(); // Draw the stroke
            this.ctx.fillStyle = 'rgba(0, 0, 0, 0.2)'; // Fill color
            this.ctx.fill(); // Fill the circle

            // Draw thumbstick
            this.ctx.beginPath();
            this.ctx.arc(this.thumbX, this.thumbY, this.thumbRadius, 0, Math.PI * 2, false);
            // Change thumb color based on NOT being in navigation mission
            if (this.currentRoverState.active_mission.toLowerCase() === 'navigation') {
                this.ctx.fillStyle = '#f84632'; /* Red for disabled/inactive */
                this.ctx.strokeStyle = '#E53935';
            } else {
                this.ctx.fillStyle = '#4CAF50'; /* Green for active */
                this.ctx.strokeStyle = '#388E3C';
            }
            this.ctx.fill();
            this.ctx.lineWidth = 2;
            this.ctx.stroke();
        }

        onMouseDown(event) {
            // Only allow dragging if the rover is NOT in 'navigation' mode
            if (this.currentRoverState.active_mission.toLowerCase() !== 'navigation') {
                this.isDragging = true;
                this.canvas.style.cursor = 'grabbing'; // Change cursor style
                this.updateThumbPosition(event); // Update thumb position immediately on click
            } else {
                // Notify user if joystick is disabled
                if (this.openmct && this.openmct.notifications) {
                    this.openmct.notifications.warn('Joystick is disabled during Navigation mission.');
                }
                console.warn('Joystick input ignored: Rover is in Navigation mission.');
            }
        }

        onMouseMove(event) {
            if (!this.isDragging) return; // Only update if actively dragging
            this.updateThumbPosition(event);
        }

        onMouseUp() {
            this.isDragging = false; // Stop dragging
            this.canvas.style.cursor = 'grab'; // Reset cursor style
            // Reset thumb to the center of the joystick base
            this.thumbX = this.joystickCenterX;
            this.thumbY = this.joystickCenterY;
            this.drawJoystick(); // Redraw to show thumb at center
            this.publishJoy(0, 0); // Send zero input to stop rover movement
        }

        // The onMouseLeave function is intentionally removed to keep the thumb
        // in position even if the cursor leaves the canvas, until mouseUp.

        updateThumbPosition(event) {
            const rect = this.canvas.getBoundingClientRect();
            // Calculate mouse position relative to the canvas
            let mouseX = event.clientX - rect.left;
            let mouseY = event.clientY - rect.top;

            // Calculate distance from the joystick's center
            let dx = mouseX - this.joystickCenterX;
            let dy = mouseY - this.joystickCenterY;
            let distance = Math.sqrt(dx * dx + dy * dy);

            // Clamp thumb position within the joystick's outer circle boundary
            if (distance > this.joystickRadius) {
                let angle = Math.atan2(dy, dx); // Angle from center to mouse position
                this.thumbX = this.joystickCenterX + this.joystickRadius * Math.cos(angle);
                this.thumbY = this.joystickCenterY + this.joystickRadius * Math.sin(angle);
            } else {
                this.thumbX = mouseX;
                this.thumbY = mouseY;
            }

            this.drawJoystick(); // Redraw joystick with the new thumb position

            // Normalize values to -1 to 1 range for ROS messages
            // X-axis (angular speed): -1 (left) to 1 (right)
            // Y-axis (linear speed): -1 (down) to 1 (up) - Y is inverted for typical ROS joystick conventions
            const normalizedX = (this.thumbX - this.joystickCenterX) / this.joystickRadius;
            const normalizedY = -(this.thumbY - this.joystickCenterY) / this.joystickRadius;

            this.publishJoy(normalizedX, normalizedY);
        }

        connectToROS() {
            // Check if ROSLIB is loaded before attempting connection
            if (typeof window.ROSLIB === 'undefined') {
                console.error("ROSLIB is not defined. Please ensure roslib.min.js is loaded.");
                this.joystickStatus.textContent = 'ROSLIB not found!';
                this.joystickStatus.classList.add('error');
                return;
            }

            // Initialize ROS connection
            this.ros = new window.ROSLIB.Ros({
                url: 'ws://localhost:9090' // Default rosbridge_server websocket URL
            });

            // Set up ROS event listeners
            this.ros.on('connection', this.handleRosConnection);
            this.ros.on('error', this.handleRosError);
            this.ros.on('close', this.handleRosClose);

            // Initialize the publisher for raw joystick data
            this.joyTopicPublisher = new window.ROSLIB.Topic({
                ros: this.ros,
                name: '/joystick/raw_input', // Topic for raw joystick input
                messageType: 'sensor_msgs/Joy' // Standard ROS Joy message type
            });

            // Subscribe to Rover Status topic from Supervisor to manage joystick activation
            this.roverStatusSubscriber = new window.ROSLIB.Topic({
                ros: this.ros,
                name: '/rover_status',
                messageType: 'roar_msgs/RoverStatus' // Ensure this message type is defined
            });
            this.roverStatusSubscriber.subscribe(this.handleRoverStatus);
        }

        handleRosConnection() {
            console.log('Connected to ROS websocket server.');
            this.rosConnected = true;
            this.updateJoystickUIState(); // Update UI on successful ROS connection
        }

        handleRosError(error) {
            console.error('Error connecting to ROS websocket server: ', error);
            this.rosConnected = false;
            this.updateJoystickUIState(); // Update UI on ROS error
        }

        handleRosClose() {
            console.log('Connection to ROS websocket server closed.');
            this.rosConnected = false;
            this.updateJoystickUIState(); // Update UI on ROS disconnect
            // Attempt to reconnect after a delay
            setTimeout(() => {
                console.log('Attempting to reconnect to ROS...');
                this.connectToROS();
            }, 3000); // Reconnect after 3 seconds
        }

        // Handler for incoming RoverStatus messages
        handleRoverStatus(message) {
            this.currentRoverState = {
                rover_state: message.rover_state,
                active_mission: message.active_mission
            };
            console.log("Rover Status Received:", this.currentRoverState);
            this.updateJoystickUIState(); // Update UI based on the new rover state
        }

        // Method to update joystick UI and enable/disable publishing based on mission
        updateJoystickUIState() {
            // Joystick is NOT active during 'navigation' mission
            const isNavigationMode = this.currentRoverState.active_mission.toLowerCase() === 'navigation';
            const isJoystickActive = !isNavigationMode; // Joystick is active if NOT in navigation mode
            
            // Update joystick thumb color based on active state
            this.drawJoystick(); 

            // Update ROS connection/mission status text
            if (this.joystickStatus) {
                if (!this.rosConnected) {
                    this.joystickStatus.textContent = 'ROS Disconnected';
                    this.joystickStatus.classList.remove('connected');
                    this.joystickStatus.classList.add('error');
                } else if (isJoystickActive) {
                    this.joystickStatus.textContent = 'Joystick Active';
                    this.joystickStatus.classList.remove('error');
                    this.joystickStatus.classList.add('connected');
                } else {
                    this.joystickStatus.textContent = `Joystick Inactive (Mission: ${this.currentRoverState.active_mission || 'None'})`;
                    this.joystickStatus.classList.remove('connected');
                    this.joystickStatus.classList.add('error');
                }
            }

            // Disable/enable sliders based on joystick active state
            if (this.linearSpeedSlider) {
                this.linearSpeedSlider.disabled = !isJoystickActive;
            }
            if (this.angularSpeedSlider) {
                this.angularSpeedSlider.disabled = !isJoystickActive;
            }

            // Update general control message below sliders
            if (this.joystickControlMessage) {
                if (isJoystickActive) {
                    this.joystickControlMessage.textContent = 'Use the joystick to control rover movement.';
                    this.joystickControlMessage.style.color = 'var(--color-text)'; // Default text color
                } else {
                    this.joystickControlMessage.textContent = `Joystick is inactive during the 'Navigation' mission.`;
                    this.joystickControlMessage.style.color = 'var(--color-error)'; // Warning/Error color
                }
            }
        }

        // Publishes joystick commands to the ROS topic
        publishJoy(normalizedX, normalizedY) {
            // Only publish if connected to ROS AND NOT in navigation mission
            if (!this.rosConnected || !this.joyTopicPublisher || this.currentRoverState.active_mission.toLowerCase() === 'navigation') {
                // If dragging, log a warning (e.g., if user tries to move joystick while it's blocked)
                if (this.isDragging) {
                    console.warn('Joystick commands not published: Rover is in Navigation mission or ROS disconnected.');
                }
                return;
            }

            // Create a standard ROS Joy message
            const joyMsg = new window.ROSLIB.Message({
                header: {
                    stamp: {
                        secs: Math.floor(Date.now() / 1000), // Current time in seconds
                        nsecs: (Date.now() % 1000) * 1e6 // Remaining nanoseconds
                    },
                    frame_id: '' // Frame ID (can be empty for simple joystick)
                },
                axes: [normalizedX, normalizedY], // Angular speed (X) and Linear speed (Y)
                buttons: [] // No buttons from this simple joystick view
            });

            this.joyTopicPublisher.publish(joyMsg);
            // console.log(`Published Joy: Axes[0]=${normalizedX.toFixed(2)}, Axes[1]=${normalizedY.toFixed(2)}`);
        }

        // This method is crucial for cleaning up resources when the view is destroyed by OpenMCT
        destroy() {
            console.log('Destroying JoystickView...');
            this.removeEventListeners(); // Remove all event listeners

            if (this.ros) {
                // Remove ROS event listeners to prevent memory leaks
                this.ros.off('connection', this.handleRosConnection);
                this.ros.off('error', this.handleRosError);
                this.ros.off('close', this.handleRosClose);
                // Unsubscribe from RoverStatus topic
                if (this.roverStatusSubscriber) {
                    this.roverStatusSubscriber.unsubscribe();
                }
                // Close the ROS connection if it's still open
                if (this.ros.isConnected) {
                    this.ros.close();
                }
            }

            // Nullify references to DOM elements and other objects to aid garbage collection
            this.canvas = null;
            this.ctx = null;
            this.linearSpeedSlider = null;
            this.angularSpeedSlider = null;
            this.linearSpeedValueSpan = null;
            this.angularSpeedValueSpan = null;
            this.joystickStatus = null;
            this.joystickControlMessage = null; // Also clear this reference
            this.htmlContent = null;
            this.roverStatusSubscriber = null;
            this.currentRoverState = null;
            this.ros = null;
            this.joyTopicPublisher = null;

            this.element.innerHTML = ''; // Clear the DOM element's content
        }
    }

    // Expose JoystickView globally so OpenMCT can find it
    window.JoystickView = JoystickView;

})();