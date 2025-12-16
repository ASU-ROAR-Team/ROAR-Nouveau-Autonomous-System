/*****************************************************************************
 * Open MCT, Copyright (c) 2014-2024, United States Government
 * as represented by the Administrator of the National Aeronautics and Space
 * All rights reserved.
 *
 * Open MCT is licensed under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 * Open MCT includes source code licensed under additional open source
 * licenses. See the Open Source Licenses file (LICENSES.md) included with
 * this source code distribution or the Licensing information page available
 * at runtime from the About dialog for additional information.
 *****************************************************************************/

// Note: Ensure ROSLIB is loaded before this plugin (e.g., in index.html)
// ROSLIB will be available globally as 'ROSLIB'

// Define the key for your new object type
const COSTMAP_MAP_KEY = 'costmap-map';

// Define the plugin factory function
// Assign the function to the window object to make it globally accessible
window.CostmapMapPlugin = function CostmapMapPlugin(options) {
    return function install(openmct) {
        // --- Define the new object type ---
        openmct.types.addType(COSTMAP_MAP_KEY, {
            name: 'Costmap Map',
            description: 'Displays a map based on a costmap and visualizes robot state.',
            creatable: true,
            cssClass: 'icon-map',
            initialize(domainObject) {
                // Initialize properties for the object
                domainObject.costmapSourceUrl = options?.defaultCostmapSourceUrl || '';

                // Properties for coordinate transformation constants
                domainObject.pixelOffsetX = options?.defaultPixelOffsetX || 208; // X offset in pixels
                domainObject.pixelOffsetY = options?.defaultPixelOffsetY || 761; // Y offset in pixels (constant in your formula)
                domainObject.pixelsPerMeter = options?.defaultPixelsPerMeter || 20.2; // Scaling factor (pixels per meter)

                // Property for Start Point
                domainObject.startPointData = options?.startPointData || [18.6625, 10.8159]; // Default to your specified start point

                // Properties for Checkpoints and Final Goal
                domainObject.checkpointsData = options?.checkpointsData || []; // Default to empty array
                domainObject.finalGoalData = options?.finalGoalData || null; // Default to null for a single point

                // Property for Landmarks
                domainObject.landmarksData = options?.landmarksData || []; // Default to empty array
            },
            form: [
                {
                    key: 'costmapSourceUrl',
                    name: 'Costmap Source URL (Image or CSV)',
                    control: 'textfield',
                    required: true,
                    cssClass: 'l-input'
                },
                {
                    key: 'pixelOffsetX',
                    name: 'Pixel Offset X',
                    control: 'numberfield',
                    required: true,
                    cssClass: 'l-input'
                },
                 {
                     key: 'pixelOffsetY',
                     name: 'Pixel Offset Y',
                     control: 'numberfield',
                     required: true,
                     cssClass: 'l-input'
                 },
                 {
                     key: 'pixelsPerMeter',
                     name: 'Pixels Per Meter',
                     control: 'numberfield',
                     required: true,
                     cssClass: 'l-input'
                 },
                 // Form field for Start Point
                 {
                     key: 'startPointData',
                     name: 'Start Point (JSON array [x, y])',
                     control: 'textfield',
                     required: false,
                     cssClass: 'l-input'
                 },
                 // Form fields for Checkpoints and Final Goal
                 {
                     key: 'checkpointsData',
                     name: 'Checkpoints (JSON array of [x, y])',
                     control: 'textarea', // Use textarea for multi-line JSON input
                     required: false,
                     cssClass: 'l-input'
                 },
                 {
                     key: 'finalGoalData',
                     name: 'Final Goal (JSON array [x, y])',
                     control: 'textfield', // Use textfield for single point JSON input
                     required: false,
                     cssClass: 'l-input'
                 },
                 // Form field for Landmarks
                 {
                     key: 'landmarksData',
                     name: 'Landmarks (JSON array of [x, y])',
                     control: 'textarea', // Use textarea for multi-line JSON input
                     required: false,
                     cssClass: 'l-input'
                 }
            ]
        });
        // --- End Define new object type ---


        // --- Define the view provider for the new object type ---
        openmct.objectViews.addProvider({
            key: 'costmap-map-view',
            name: 'Costmap Map View',
            canView: (domainObject) => {
                return domainObject.type === COSTMAP_MAP_KEY;
            },
            view: (domainObject, objectPath) => {
                let mapComponent = null;

                return {
                    show(element, editMode) {
                        mapComponent = new CostmapMapComponent(element, domainObject, openmct);
                        mapComponent.render();
                    },
                    onEditModeChange(editMode) {
                        // Optional: Handle changes to edit mode
                    },
                    destroy: function (element) {
                        if (mapComponent && typeof mapComponent.destroy === 'function') {
                            mapComponent.destroy();
                        }
                        mapComponent = null;
                    }
                };
            }
        });
        // --- End Define view provider ---

        return {
            destroy: () => { }
        };
    };
}


// --- 2. Create the Visualization Component (CostmapMapComponent) ---
// This component handles loading the costmap and drawing dynamic elements on a Canvas.

class CostmapMapComponent {
    constructor(parentElement, domainObject, openmct) {
        this.parentElement = parentElement;
        this.domainObject = domainObject;
        this.openmct = openmct;

        this.canvas = null;
        this.ctx = null;
        this.costmapSourceType = null; // 'image' or 'csv'
        this.costmapImage = null; // For image source
        this.costmapData = null; // For CSV source (2D array)

        // Coordinate transformation constants loaded from the domain object
        this.pixelOffsetX = this.domainObject.pixelOffsetX;
        this.pixelOffsetY = this.domainObject.pixelOffsetY;
        this.pixelsPerMeter = this.domainObject.pixelsPerMeter;

        this.mapWidthPixels = 0; // Determined by source (image width or CSV columns)
        this.mapHeightPixels = 0; // Determined by source (image height or CSV rows)
        this.minCost = 0; // For CSV normalization
        this.maxCost = 0; // For CSV normalization


        // ROS connection (assuming ROSLIB is available globally)
        this.ros = null;

        // ROS Topics and Subscribers (replace with your actual topics)
        this.robotPoseTopic = null;
        this.pathTopic = null;
        this.traversedPathTopic = null;
        this.lookaheadTopic = null;
        this.obstaclesTopic = null;

        // Data storage for visualization elements (will be cleared on ROS disconnect)
        this.robotPosition = null; // {x, y, theta}
        this.globalPath = [];      // [{x, y}]
        this.traversedPath = [];   // [{x, y}]
        this.lookaheadPoint = null; // {x, y}
        this.obstacles = new Map(); // Map<obstacle_id, {x, y, radius}>

        // Static points from properties
        this.startPoint = this.processStartPointData(this.domainObject.startPointData);
        this.checkpoints = this.processCheckpointsData(this.domainObject.checkpointsData);
        this.finalGoal = this.processFinalGoalData(this.domainObject.finalGoalData);
        this.landmarks = this.processLandmarksData(this.domainObject.landmarksData);

        // Bind event handlers
        this.resizeHandler = this.handleResize.bind(this);

        // Variables for browser-side rate limiting for pose updates
        this.lastProcessedPoseTime = 0;
        this.minProcessingIntervalMs = 33; // ~30 fps

        // Timestamps for data staleness check
        this.lastRobotPoseReceiveTime = 0;
        this.lastGlobalPathReceiveTime = 0;
        this.lastTraversedPathReceiveTime = 0;
        this.lastObstaclesReceiveTime = 0;
        this.dataStaleTimeoutMs = 2000; // 2 seconds threshold for data to be considered stale (adjust as needed)

        // The animation frame ID for the continuous drawing loop
        this.animationFrameId = null;
    }

    // --- Methods to process data from properties (handling potential JSON strings) ---

    // Method to process Start Point data
    processStartPointData(data) {
        let processedData = null;
         if (typeof data === 'string') {
             try {
                 processedData = JSON.parse(data);
             } catch (error) {
                 console.error('Map: Error parsing start point JSON string:', error, data);
                 return null;
             }
         } else if (Array.isArray(data) && data.length === 2 && typeof data[0] === 'number' && typeof data[1] === 'number') {
              processedData = data;
         } else if (data === null || data === undefined) {
              return null;
         }
         else {
             return null;
         }

         if (Array.isArray(processedData) && processedData.length === 2 && typeof processedData[0] === 'number' && typeof processedData[1] === 'number') {
             return { x: processedData[0], y: processedData[1] };
         } else {
             console.warn('Map: Invalid start point data format after processing. Expected [x, y] array.', processedData);
             return null;
         }
    }

    processCheckpointsData(data) {
        let processedData = [];
        if (typeof data === 'string') {
            try {
                processedData = JSON.parse(data);
            } catch (error) {
                console.error('Map: Error parsing checkpoints JSON string:', error, data);
                return []; // Return empty array on parsing error
            }
        } else if (Array.isArray(data)) {
             processedData = data;
        } else {
            return []; // Return empty array if data is not string or array
        }

        // Basic validation: check if it's an array of arrays with 2 numbers
        if (Array.isArray(processedData) && processedData.every(cp => Array.isArray(cp) && cp.length === 2 && typeof cp[0] === 'number' && typeof cp[1] === 'number')) {
            return processedData.map(cp => ({ x: cp[0], y: cp[1] })); // Convert to {x, y} objects
        } else {
            console.warn('Map: Invalid checkpoints data format after processing. Expected array of [x, y] arrays.', processedData);
            return []; // Return empty array on invalid format
        }
    }

    processFinalGoalData(data) {
        let processedData = null;
         if (typeof data === 'string') {
             try {
                 processedData = JSON.parse(data);
             } catch (error) {
                 console.error('Map: Error parsing final goal JSON string:', error, data);
                 return null; // Return null on parsing error
             }
         } else if (Array.isArray(data) && data.length === 2 && typeof data[0] === 'number' && typeof data[1] === 'number') {
              processedData = data; // Already in the expected array format
         } else if (data === null || data === undefined) {
              return null; // Explicitly return null if input is null/undefined
         }
         else {
             return null; // Return null if data is not string or array or invalid format
         }

         // Basic validation: check if it's an array with 2 numbers
         if (Array.isArray(processedData) && processedData.length === 2 && typeof processedData[0] === 'number' && typeof processedData[1] === 'number') {
             return { x: processedData[0], y: processedData[1] }; // Convert to {x, y} object
         } else {
             console.warn('Map: Invalid final goal data format after processing. Expected [x, y] array.', processedData);
             return null; // Return null on invalid format
         }
    }

    processLandmarksData(data) {
        let processedData = [];
        if (typeof data === 'string') {
            try {
                processedData = JSON.parse(data);
            } catch (error) {
                console.error('Map: Error parsing landmarks JSON string:', error, data);
                return []; // Return empty array on parsing error
            }
        } else if (Array.isArray(data)) {
            processedData = data;
        } else {
             return []; // Return empty array if data is not string or array
        }

        // Basic validation: check if it's an array of arrays with 2 numbers
        if (Array.isArray(processedData) && processedData.every(lm => Array.isArray(lm) && lm.length === 2 && typeof lm[0] === 'number' && typeof lm[1] === 'number')) {
            return processedData.map(lm => ({ x: lm[0], y: lm[1] })); // Convert to {x, y} objects
        } else {
            console.warn('Map: Invalid landmarks data format after processing. Expected array of [x, y] arrays.', processedData);
            return []; // Return empty array on invalid format
        }
    }

    render() {
        this.canvas = document.createElement('canvas');
        this.canvas.style.width = '100%';
        this.canvas.style.height = '100%';
        this.parentElement.appendChild(this.canvas);
        this.ctx = this.canvas.getContext('2d');

        this.handleResize();
        window.addEventListener('resize', this.resizeHandler);

        // Determine costmap source type and load accordingly
        if (this.domainObject.costmapSourceUrl.toLowerCase().endsWith('.csv')) {
            this.costmapSourceType = 'csv';
            this.loadCostmapData(this.domainObject.costmapSourceUrl);
        } else if (this.domainObject.costmapSourceUrl) {
             this.costmapSourceType = 'image';
             this.loadCostmapImage();
        } else {
             console.error('Map: Costmap Source URL is not provided in object properties.');
             if (this.ctx) {
                 this.ctx.fillStyle = 'red';
                 this.ctx.font = '20px Arial';
                 this.ctx.fillText('Costmap Source URL not set.', 10, 30);
             }
        }

        // Setup ROS connection and subscribers
        this.setupRos();

        // Set up a periodic drawing and staleness check loop
        this.animationFrameId = requestAnimationFrame(this.animate.bind(this));
    }

    animate() {
        this.checkStaleData(); // Check if any data has gone stale
        this.drawMap();        // Redraw the map (this will only draw if data is present and not stale)
        this.animationFrameId = requestAnimationFrame(this.animate.bind(this)); // Request next frame
    }

    handleResize() {
        // Adjust canvas drawing buffer size to match display size
        const rect = this.canvas.parentElement.getBoundingClientRect();
        this.canvas.width = rect.width;
        // Maintain aspect ratio if map dimensions are known
        if (this.mapWidthPixels > 0 && this.mapHeightPixels > 0) {
            this.canvas.height = rect.width * (this.mapHeightPixels / this.mapWidthPixels);
            if (this.canvas.height > rect.height) { // If calculated height exceeds parent's height, scale by height
                this.canvas.height = rect.height;
                this.canvas.width = rect.height * (this.mapWidthPixels / this.mapHeightPixels);
            }
        } else {
            this.canvas.height = rect.height; // Fallback to parent height if map dimensions unknown
        }

        // Redraw the map after resizing
        this.drawMap();
    }

    loadCostmapImage() {
        this.costmapImage = new Image();
        this.costmapImage.src = this.domainObject.costmapSourceUrl; // Use URL from properties
        this.costmapImage.onload = () => {
            this.mapWidthPixels = this.costmapImage.width;
            this.mapHeightPixels = this.costmapImage.height;
            this.handleResize(); // Recalculate canvas dimensions and redraw
        };
        this.costmapImage.onerror = (error) => {
            console.error('Map: Error loading costmap image:', error, this.domainObject.costmapSourceUrl);
            if (this.ctx) {
                this.ctx.fillStyle = 'red';
                this.ctx.font = '20px Arial';
                this.ctx.fillText(`Error loading map image: ${this.domainObject.costmapSourceUrl}`, 10, 30);
            }
        };
    }

    async loadCostmapData(csvUrl) {
        try {
            const response = await fetch(csvUrl);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            const csvText = await response.text();

            // Simple manual parsing (consider a library for robustness)
            const rows = csvText.trim().split('\n');
            this.costmapData = rows.map(row => row.split(',').map(Number)); // Assuming comma delimiter and numbers

            this.mapHeightPixels = this.costmapData.length;
            if (this.mapHeightPixels > 0) {
                 this.mapWidthPixels = this.costmapData[0].length;
            } else {
                 this.mapWidthPixels = 0;
            }

            if (this.mapWidthPixels === 0 || this.mapHeightPixels === 0) {
                 console.error('Map: Costmap data is empty or invalid.');
                 if (this.ctx) {
                     this.ctx.fillStyle = 'red';
                     this.ctx.font = '20px Arial';
                     this.ctx.fillText('Invalid costmap data.', 10, 30);
                 }
                 return;
            }

            // Find min/max cost for normalization
            this.minCost = Infinity;
            this.maxCost = -Infinity;
            for (const row of this.costmapData) {
                 for (const value of row) {
                     if (value < this.minCost) this.minCost = value;
                     if (value > this.maxCost) this.maxCost = value;
                 }
            }

            this.handleResize(); // Recalculate canvas dimensions and redraw

        } catch (error) {
            console.error('Map: Error loading or parsing costmap CSV:', error);
            if (this.ctx) {
                this.ctx.fillStyle = 'red';
                this.ctx.font = '20px Arial';
                this.ctx.fillText('Error loading costmap CSV.', 10, 30);
            }
        }
    }

    // Function to get color from cost value
    getCostColor(costValue) {
        // Handle case where min and max cost are the same or invalid range
        if (this.maxCost <= this.minCost) {
             // If all valid costs are the same or range is invalid, return a mid-gray
             if (costValue === -1) {
                return 'rgba(100, 100, 100, 0.5)'; // Semi-transparent gray for -1 (unknown/unoccupied)
             }
             return 'rgb(128, 128, 128)'; // Mid-gray for other cases (e.g., all costs are 0)
        }

        // Normalize cost to 0-1 range
        const normalizedCost = (costValue - this.minCost) / (this.maxCost - this.minCost);

        // Implement a simple colormap (Blue for low cost to Red for high cost)
        const r = Math.floor(normalizedCost * 255); // Red increases with cost
        const g = 0; // Green is zero for pure blue-red gradient
        const b = Math.floor((1 - normalizedCost) * 255); // Blue decreases with cost

        return `rgb(${r}, ${g}, ${b})`;
    }

    // --- Coordinate Transformation Functions (Based on your Python logic) ---
    // These functions convert real-world ROS coordinates (meters) to pixel coordinates
    // relative to the top-left of the costmap data/image, using your constants.
    realToPixelX(realX) {
        return this.pixelOffsetX + this.pixelsPerMeter * realX;
    }

    realToPixelY(realY) {
        return this.pixelOffsetY - this.pixelsPerMeter * realY; // Subtract because canvas Y is inverted
    }

    // --- Drawing Function ---
    drawMap() {
        if (!this.ctx || !this.canvas) {
            console.warn('Map: Canvas context not available for drawing.');
            return;
        }

        // Clear the canvas
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        // Determine scaling factors from costmap dimensions to current canvas size
        const scaleX = this.mapWidthPixels > 0 ? this.canvas.width / this.mapWidthPixels : 1;
        const scaleY = this.mapHeightPixels > 0 ? this.canvas.height / this.mapHeightPixels : 1;

        // For markers, use the smaller scale factor to maintain relative proportion on the map
        const markerScale = Math.min(scaleX, scaleY);


        // --- Draw the costmap background (Image or CSV pixels) ---
        if (this.costmapSourceType === 'image' && this.costmapImage && this.costmapImage.complete) {
            // Draw the image, scaled to fill the canvas
            this.ctx.drawImage(this.costmapImage, 0, 0, this.canvas.width, this.canvas.height);
        } else if (this.costmapSourceType === 'csv' && this.costmapData && this.mapWidthPixels > 0 && this.mapHeightPixels > 0) {
            // Draw the costmap pixel by pixel from CSV data
            for (let y = 0; y < this.mapHeightPixels; y++) {
                for (let x = 0; x < this.mapWidthPixels; x++) {
                    const cost = this.costmapData[y][x];
                    const color = this.getCostColor(cost);
                    this.ctx.fillStyle = color;
                    // Draw a rectangle for each pixel, scaled to the canvas size
                    this.ctx.fillRect(x * scaleX, y * scaleY, scaleX, scaleY);
                }
            }
        } else {
            // Draw a placeholder or message if data is not loaded
             this.ctx.fillStyle = 'gray';
             this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
             this.ctx.fillStyle = 'white';
             this.ctx.font = '20px Arial';
             this.ctx.fillText('Loading map data or URL not set...', 10, 30);
        }

        // --- Draw other elements (convert ROS coords to pixel coords and scale) ---
        // Draw Global Path
        if (this.globalPath.length > 1) {
            this.ctx.strokeStyle = 'yellow';
            this.ctx.lineWidth = 2;
            this.ctx.beginPath();
            const startPixel = {
                x: this.realToPixelX(this.globalPath[0].x) * scaleX,
                y: this.realToPixelY(this.globalPath[0].y) * scaleY
            };
            this.ctx.moveTo(startPixel.x, startPixel.y);
            for (let i = 1; i < this.globalPath.length; i++) {
                const nextPixel = {
                    x: this.realToPixelX(this.globalPath[i].x) * scaleX,
                    y: this.realToPixelY(this.globalPath[i].y) * scaleY
                };
                this.ctx.lineTo(nextPixel.x, nextPixel.y);
            }
            this.ctx.stroke();
        }

       // Draw Traversed Path
        if (this.traversedPath.length > 1) {
             this.ctx.strokeStyle = 'blue';
             this.ctx.lineWidth = 2;
             this.ctx.beginPath();
             const startPixel = {
                 x: this.realToPixelX(this.traversedPath[0].x) * scaleX,
                 y: this.realToPixelY(this.traversedPath[0].y) * scaleY
             };
             this.ctx.moveTo(startPixel.x, startPixel.y);
             for (let i = 1; i < this.traversedPath.length; i++) {
                 const nextPixel = {
                     x: this.realToPixelX(this.traversedPath[i].x) * scaleX,
                     y: this.realToPixelY(this.traversedPath[i].y) * scaleY
                 };
                 this.ctx.lineTo(nextPixel.x, nextPixel.y);
             }
             this.ctx.stroke();
        }

        // Draw Robot Position - Will disappear if this.robotPosition becomes null (due to staleness or explicit 'not found')
        if (this.robotPosition) {
            const robotPixel = {
                x: this.realToPixelX(this.robotPosition.x) * scaleX,
                y: this.realToPixelY(this.robotPosition.y) * scaleY
            };
            this.ctx.fillStyle = 'red';
            this.ctx.beginPath();
            this.ctx.arc(robotPixel.x, robotPixel.y, 5 * markerScale, 0, Math.PI * 2); // Scaled radius
            this.ctx.fill();

            // Draw orientation line
            const lineLength = 10 * markerScale;
            const endX = robotPixel.x + lineLength * Math.cos(this.robotPosition.theta);
            const endY = robotPixel.y - lineLength * Math.sin(this.robotPosition.theta); // Canvas Y is inverted
            this.ctx.strokeStyle = 'black';
            this.ctx.lineWidth = 2 * markerScale;
            this.ctx.beginPath();
            this.ctx.moveTo(robotPixel.x, robotPixel.y);
            this.ctx.lineTo(endX, endY);
            this.ctx.stroke();
        }

        // Draw Lookahead Point
        if (this.lookaheadPoint) {
            const lookaheadPixel = {
                x: this.realToPixelX(this.lookaheadPoint.x) * scaleX,
                y: this.realToPixelY(this.lookaheadPoint.y) * scaleY
            };
            const lookaheadSize = 8 * markerScale;
            this.ctx.fillStyle = 'cyan';
            this.ctx.fillRect(lookaheadPixel.x - lookaheadSize / 2, lookaheadPixel.y - lookaheadSize / 2, lookaheadSize, lookaheadSize);
        }

        // Draw Obstacles
        if (this.obstacles.size > 0) {
             this.obstacles.forEach(obstacle => {
                 const obstaclePixel = {
                     x: this.realToPixelX(obstacle.x) * scaleX,
                     y: this.realToPixelY(obstacle.y) * scaleY
                 };
                 const pixelRadius = obstacle.radius * this.pixelsPerMeter * markerScale; // Scale radius
                 this.ctx.fillStyle = 'rgba(255, 0, 0, 0.5)'; // Red with some transparency
                 this.ctx.beginPath();
                 this.ctx.arc(obstaclePixel.x, obstaclePixel.y, pixelRadius, 0, Math.PI * 2);
                 this.ctx.fill();
             });
        }
        
        // Draw Start Point
        if (this.startPoint) {
            const startPointPixel = {
                x: this.realToPixelX(this.startPoint.x) * scaleX,
                y: this.realToPixelY(this.startPoint.y) * scaleY
            };
            this.ctx.fillStyle = '#00FF00'; // Bright green
            this.ctx.strokeStyle = 'black';
            const baseRadius = 6;
            const radius = Math.max(3, baseRadius * markerScale * 0.7);
            this.ctx.lineWidth = Math.max(1, 2 * markerScale * 0.5);

            this.ctx.beginPath();
            this.ctx.arc(startPointPixel.x, startPointPixel.y, radius, 0, Math.PI * 2);
            this.ctx.fill();
            this.ctx.stroke();
        }

        // Draw Checkpoints (from properties) - Re-introduced controlled scaling, but smaller
        if (Array.isArray(this.checkpoints) && this.checkpoints.length > 0) {
             this.checkpoints.forEach((checkpoint) => {
                  const checkpointPixel = {
                      x: this.realToPixelX(checkpoint.x) * scaleX,
                      y: this.realToPixelY(checkpoint.y) * scaleY
                  };
                  // Controlled scaling for fixed-size markers - SMALLER
                  const baseTriangleSize = 10; // Base size in pixels (was 15)
                  const triangleSize = Math.max(4, baseTriangleSize * markerScale * 0.6); // Apply a gentler scale, ensure min size (was 5, 0.7)

                  this.ctx.fillStyle = 'magenta';
                  this.ctx.beginPath();
                  this.ctx.moveTo(checkpointPixel.x, checkpointPixel.y - triangleSize / 2);
                  this.ctx.lineTo(checkpointPixel.x - triangleSize / 2, checkpointPixel.y + triangleSize / 2);
                  this.ctx.lineTo(checkpointPixel.x + triangleSize / 2, checkpointPixel.y + triangleSize / 2);
                  this.ctx.closePath();
                  this.ctx.fill();
             });
        }

        // Draw Landmarks (from properties) - Re-introduced controlled scaling, but smaller
        if (Array.isArray(this.landmarks) && this.landmarks.length > 0) {
             this.landmarks.forEach((landmark) => {
                  const landmarkPixel = {
                      x: this.realToPixelX(landmark.x) * scaleX,
                      y: this.realToPixelY(landmark.y) * scaleY
                  };
                  // Controlled scaling for fixed-size markers - SMALLER
                  const baseSquareSize = 8; // Base size in pixels (was 12)
                  const squareSize = Math.max(3, baseSquareSize * markerScale * 0.6); // (was 4, 0.7)

                  this.ctx.fillStyle = 'green';
                  this.ctx.fillRect(landmarkPixel.x - squareSize / 2, landmarkPixel.y - squareSize / 2, squareSize, squareSize);
             });
        }

        // Draw Final Goal (from properties) - Re-introduced controlled scaling, but smaller
        if (this.finalGoal) {
             const finalGoalPixel = {
                 x: this.realToPixelX(this.finalGoal.x) * scaleX,
                 y: this.realToPixelY(this.finalGoal.y) * scaleY
             };
             this.ctx.strokeStyle = 'lime';
             // Controlled line width - SMALLER
             const baseLineWidth = 2; // (was 3)
             this.ctx.lineWidth = Math.max(1, baseLineWidth * markerScale * 0.4); // (was 1, 0.5)

             // Controlled size for the 'X' - SMALLER
             const baseXSize = 14; // Base size in pixels for the 'X' (was 18)
             const xSize = Math.max(6, baseXSize * markerScale * 0.6); // Adjust for visual balance (was 8, 0.7)

             this.ctx.beginPath();
             this.ctx.moveTo(finalGoalPixel.x - xSize / 2, finalGoalPixel.y - xSize / 2);
             this.ctx.lineTo(finalGoalPixel.x + xSize / 2, finalGoalPixel.y + xSize / 2);
             this.ctx.stroke();

             this.ctx.beginPath();
             this.ctx.moveTo(finalGoalPixel.x + xSize / 2, finalGoalPixel.y - xSize / 2);
             this.ctx.lineTo(finalGoalPixel.x - xSize / 2, finalGoalPixel.y + xSize / 2);
             this.ctx.stroke();
        }
    }

    // NEW: checkStaleData method
    checkStaleData() {
        const now = performance.now();

        // Check Robot Pose
        if (this.robotPosition && (now - this.lastRobotPoseReceiveTime > this.dataStaleTimeoutMs)) {
            console.warn('Map: Robot pose data is stale. Clearing robot display.');
            this.robotPosition = null;
            this.lastRobotPoseReceiveTime = 0; // Reset timestamp
        }

        // Check Global Path
        if (this.globalPath.length > 0 && (now - this.lastGlobalPathReceiveTime > this.dataStaleTimeoutMs)) {
            console.warn('Map: Global path data is stale. Clearing path display.');
            this.globalPath = [];
            this.lastGlobalPathReceiveTime = 0; // Reset timestamp
        }

        // Check Traversed Path
        if (this.traversedPath.length > 0 && (now - this.lastTraversedPathReceiveTime > this.dataStaleTimeoutMs)) {
            console.warn('Map: Traversed path data is stale. Clearing path display.');
            this.traversedPath = [];
            this.lastTraversedPathReceiveTime = 0; // Reset timestamp
        }

        // Check Obstacles
        if (this.obstacles.size > 0 && (now - this.lastObstaclesReceiveTime > this.dataStaleTimeoutMs)) {
             console.warn('Map: Obstacle data is stale. Clearing obstacle display.');
             this.obstacles.clear();
             this.lastObstaclesReceiveTime = 0; // Reset timestamp
        }
        // Note: No need to call drawMap() here, as animate() calls it right after.
    }

    // --- ROS Connection and Topic Setup ---
    setupRos() {
        // Setup ROS connection if it's not already established
        if (!this.ros && typeof ROSLIB !== 'undefined') {
            try {
                 this.ros = new ROSLIB.Ros({
                     url: 'ws://localhost:9090' // Match your rosbridge_websocket URL
                 });

                 this.ros.on('connection', () => {
                     console.log('Map: ROS Connected.');
                     this.setupRosTopics(); // Setup topics once connected
                 });

                 this.ros.on('error', (error) => {
                    console.error('Map: ROS Connection error:', error);
                    // On connection error, assume all data streams stop
                    this.clearDynamicData(); // Clear stale data on error
                    this.drawMap(); // Redraw to clear visualization
                 });

                 this.ros.on('close', () => {
                     console.warn('Map: ROS Connection closed. Clearing dynamic data.');
                     this.clearDynamicData(); // Clear the stored data
                     this.drawMap(); // Redraw the map to show cleared data
                 });

            } catch (error) {
                 console.error('Map: Error setting up ROS connection:', error);
                 this.ros = null; // Ensure ros is null if connection fails
            }
        } else if (typeof ROSLIB === 'undefined') {
             console.error('Map: ROSLIB library not available. Cannot setup ROS.');
             return; // Cannot proceed without ROSLIB
        }

        // If ROS is already connected, setup topics immediately
        if (this.ros && this.ros.isConnected) {
             this.setupRosTopics();
        } else if (this.ros) {
             console.warn('Map: ROS connection not established. Waiting for connection to setup topics.');
         }
    }

    setupRosTopics() {
         if (!this.ros || !this.ros.isConnected) {
              console.warn('Map: ROS not connected, cannot setup topics.');
              return;
         }

         // --- Setup Subscribers for relevant ROS topics ---
         // MODIFIED: Robot Pose subscriber
         this.robotPoseTopic = new ROSLIB.Topic({
             ros: this.ros,
             name: '/filtered_state', // MODIFIED: Topic name
             messageType: 'nav_msgs/Odometry' // MODIFIED: Message type
         });
         this.robotPoseTopic.subscribe(this.handleRobotPose.bind(this));
         console.log('Map: Subscribed to /filtered_state');


         // Global Path
         this.pathTopic = new ROSLIB.Topic({
             ros: this.ros,
             name: '/Path', // Replace with your planned path topic
             messageType: 'nav_msgs/Path' // Replace with your path message type
         });
         this.pathTopic.subscribe(this.handleGlobalPath.bind(this));
         console.log('Map: Subscribed to /Path');

         // Traversed Path
         this.traversedPathTopic = new ROSLIB.Topic({
             ros: this.ros,
             name: '/traversed_path', // Example topic name
             messageType: 'nav_msgs/Path' // Example message type
         });
         this.traversedPathTopic.subscribe(this.handleTraversedPath.bind(this));
         console.log('Map: Subscribed to /traversed_path');

         // Obstacles
         this.obstaclesTopic = new ROSLIB.Topic({
             ros: this.ros,
             name: '/obstacles', // Replace with your obstacle topic name
             messageType: 'roar_msgs/Obstacle' // Use the message type name
         });
         this.obstaclesTopic.subscribe(this.handleObstacles.bind(this));
         console.log('Map: Subscribed to /obstacles');

         // Lookahead Point (If published separately by APF)
         // this.lookaheadTopic = new ROSLIB.Topic({
         //    ros: this.ros,
         //    name: '/lookahead_point', // Example topic name
         //    messageType: 'geometry_msgs/PointStamped' // Or appropriate message type
         // });
         // this.lookaheadTopic.subscribe(this.handleLookahead.bind(this));
         // console.log('Map: Subscribed to /lookahead_point');
    }

    // --- Method to clear dynamic data (robot pose, paths, obstacles) ---
    clearDynamicData() {
        this.robotPosition = null;
        this.globalPath = [];
        this.traversedPath = [];
        this.lookaheadPoint = null;
        this.obstacles.clear(); // Clear the Map
        // Also reset timestamps to ensure things don't immediately reappear if any data source becomes active quickly
        this.lastRobotPoseReceiveTime = 0;
        this.lastGlobalPathReceiveTime = 0;
        this.lastTraversedPathReceiveTime = 0;
        this.lastObstaclesReceiveTime = 0;
    }

    // --- ROS Message Handlers (update data and redraw) ---
    // MODIFIED: handleRobotPose to process nav_msgs/Odometry
    handleRobotPose(msg) {
        const now = performance.now(); // Get current time for rate limiting and staleness tracking

        // Rate limiting for drawing updates
        if (now - this.lastProcessedPoseTime < this.minProcessingIntervalMs) {
            this.lastRobotPoseReceiveTime = now; // Always update receive time to prevent premature staleness
            return;
        }
        this.lastProcessedPoseTime = now;

        try {
            // Directly access pose data from the Odometry message
            if (msg.pose && msg.pose.pose) {
                const pose = msg.pose.pose;
                const orientation = pose.orientation;

                // Convert quaternion to Euler angles to get yaw (theta)
                const q = new ROSLIB.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
                let rpy = { x: 0, y: 0, z: 0 };
                // Use a reliable method to convert quaternion to euler
                if (typeof q.toEuler === 'function') {
                    rpy = q.toEuler();
                } else {
                    // Manual conversion as a fallback
                    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
                    rpy.z = Math.atan2(siny_cosp, cosy_cosp);
                }

                this.robotPosition = {
                    x: pose.position.x,
                    y: pose.position.y,
                    theta: rpy.z // Yaw is the rotation around the Z-axis
                };
                this.lastRobotPoseReceiveTime = now;

            } else {
                if (this.robotPosition !== null) {
                    this.robotPosition = null;
                    this.lastRobotPoseReceiveTime = 0;
                    console.warn(`Map: Received invalid Odometry message on /filtered_state. Clearing robot display.`);
                }
            }
        } catch (error) {
            console.error('Map: Error processing robot pose message:', error);
            if (this.robotPosition !== null) {
                this.robotPosition = null;
                this.lastRobotPoseReceiveTime = 0;
            }
        }
    }

    handleGlobalPath(msg) {
        const now = performance.now(); // Get current time
        if (Array.isArray(msg.poses) && msg.poses.length > 0) {
            this.globalPath = msg.poses.map(p => ({ x: p.pose.position.x, y: p.pose.position.y }));
            this.lastGlobalPathReceiveTime = now; // IMPORTANT: Update receive time here
            // drawMap() will be called by the animate() loop
        } else {
            // If path is empty or invalid in a *received* message, clear it immediately
            if (this.globalPath.length > 0) {
                 console.warn('Map: Received empty or invalid global path message. Clearing path.');
                 this.globalPath = [];
                 this.lastGlobalPathReceiveTime = 0; // Reset timestamp
                 // drawMap() will be called by the animate() loop
            }
        }
    }

    handleTraversedPath(msg) {
         const now = performance.now(); // Get current time
         if (Array.isArray(msg.poses) && msg.poses.length > 0) {
            this.traversedPath = msg.poses.map(p => ({ x: p.pose.position.x, y: p.pose.position.y }));
            this.lastTraversedPathReceiveTime = now; // IMPORTANT: Update receive time here
            // drawMap() will be called by the animate() loop
         } else {
            // If path is empty or invalid in a *received* message, clear it immediately
            if (this.traversedPath.length > 0) {
                 console.warn('Map: Received empty or invalid traversed path message. Clearing path.');
                 this.traversedPath = [];
                 this.lastTraversedPathReceiveTime = 0; // Reset timestamp
                 // drawMap() will be called by the animate() loop
            }
         }
    }

    handleObstacles(msg) {
        const now = performance.now(); // Get current time
        try {
            if (msg && msg.id && msg.id.data !== undefined && msg.position && msg.position.pose && msg.position.pose.position && msg.radius && msg.radius.data !== undefined) {
                const obstacleId = msg.id.data;
                const obstacleData = {
                    x: msg.position.pose.position.x,
                    y: msg.position.pose.position.y,
                    radius: msg.radius.data
                };
                this.obstacles.set(obstacleId, obstacleData);
                this.lastObstaclesReceiveTime = now; // IMPORTANT: Update receive time here
                // drawMap() will be called by the animate() loop
            } else {
                 console.warn('Map: Received obstacle message with missing or invalid fields.', msg);
            }
        } catch (error) {
            console.error('Map: Error processing Obstacle message:', error);
        }
    }

    // handleLookahead(msg) { ... } // If you enable lookahead topic, update its timestamp here
    // handleCheckpoints(msg) { ... } // If these become dynamic, update timestamps
    // handleLandmarks(msg) { ... } // If these become dynamic, update timestamps

    destroy() {
        window.removeEventListener('resize', this.resizeHandler);

        // Cancel the animation frame loop
        if (this.animationFrameId) {
            cancelAnimationFrame(this.animationFrameId);
        }

        // Unsubscribe from all ROS topics
        if (this.robotPoseTopic) this.robotPoseTopic.unsubscribe();
        if (this.pathTopic) this.pathTopic.unsubscribe();
        if (this.traversedPathTopic) this.traversedPathTopic.unsubscribe();
        if (this.lookaheadTopic) this.lookaheadTopic.unsubscribe();
        if (this.obstaclesTopic) this.obstaclesTopic.unsubscribe();

        // Clear any remaining dynamic data when the component is destroyed
        this.clearDynamicData();

        // Close ROS connection if this component is solely responsible for it
        if (this.ros && this.ros.isConnected) {
            this.ros.close();
        }
        this.ros = null;

        if (this.canvas && this.canvas.parentElement) {
            this.canvas.parentElement.removeChild(this.canvas);
        }
        this.canvas = null;
        this.ctx = null;
    }
}