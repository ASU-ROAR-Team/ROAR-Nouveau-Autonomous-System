const ZED_CAMERA_KEY = 'zed-camera';

// Define the plugin factory function
window.ZEDPlugin = function ZEDPlugin() {
    return function install(openmct) {
        // --- Define the new object type ---
        openmct.types.addType(ZED_CAMERA_KEY, {
            name: 'ZED Camera',
            description: 'Displays a video feed from a ZED 2 camera ROS topic via rosbridge_server and roslibjs.',
            creatable: true,
            cssClass: 'icon-camera',
            initialize(domainObject) {
                domainObject.rosbridgeUrl = 'ws://localhost:9090';
                domainObject.rosImageTopic = '/zed2i/zed_node/depth/depth_registered/color_mapped_image/compressed_for_web';
                domainObject.throttleRate = 200; // Increased to 200ms (5fps) - less aggressive
            },
            form: [
                {
                    key: 'rosbridgeUrl',
                    name: 'ROSBridge WebSocket URL',
                    control: 'textfield',
                    required: true,
                    cssClass: 'l-input',
                },
                {
                    key: 'rosImageTopic',
                    name: 'ROS Image Topic Name',
                    control: 'textfield',
                    required: true,
                    cssClass: 'l-input',
                },
                {
                    key: 'throttleRate',
                    name: 'Throttle Rate (ms)',
                    control: 'numberfield',
                    required: false,
                    cssClass: 'l-input',
                }
            ]
        });

        // --- Define the view provider ---
        openmct.objectViews.addProvider({
            key: 'zed-camera-view',
            name: 'ZED Camera View',
            canView: (domainObject) => {
                return domainObject.type === ZED_CAMERA_KEY;
            },
            view: (domainObject) => {
                let imgElement = null;
                let ros = null;
                let imageTopicSubscriber = null;
                let errorMessageElement = null;
                let viewContainerElement = null;
                let snapshotButton = null;
                let innerCircle = null;

                // Performance optimization flags
                let isProcessingFrame = false;
                let lastFrameTime = 0;
                let frameCount = 0;
                let startTime = Date.now();

                const displayMessage = (message, type = 'info') => {
                    // Clear previous messages
                    if (errorMessageElement && errorMessageElement.parentElement) {
                        errorMessageElement.parentElement.removeChild(errorMessageElement);
                        errorMessageElement = null;
                    }

                    // Hide image if message is an error/warning
                    if (imgElement) {
                        imgElement.style.display = (type === 'error' || type === 'warning') ? 'none' : 'block';
                    }

                    errorMessageElement = document.createElement('div');
                    errorMessageElement.style.textAlign = 'center';
                    errorMessageElement.style.marginTop = '20px';
                    errorMessageElement.style.padding = '10px';
                    errorMessageElement.style.borderRadius = '5px';

                    if (type === 'error') {
                        errorMessageElement.style.color = 'white';
                        errorMessageElement.style.backgroundColor = '#d9534f';
                    } else if (type === 'warning') {
                        errorMessageElement.style.color = 'black';
                        errorMessageElement.style.backgroundColor = '#f0ad4e';
                    } else {
                        errorMessageElement.style.color = 'black';
                        errorMessageElement.style.backgroundColor = '#d9edf7';
                    }
                    errorMessageElement.textContent = message;

                    if (viewContainerElement) {
                        viewContainerElement.appendChild(errorMessageElement);
                    } else {
                        console.error('ZED Plugin: Cannot find view container element');
                        document.body.appendChild(errorMessageElement);
                    }
                };

                const logTiming = (label) => {
                    const now = Date.now();
                    console.log(`ZED Plugin Timing - ${label}: ${now}ms (since start: ${now - startTime}ms)`);
                    return now;
                };

                const connectAndSubscribe = (rosbridgeUrl, rosImageTopic, throttleRate) => {
                    // Clean up existing connections
                    if (imageTopicSubscriber) {
                        imageTopicSubscriber.unsubscribe();
                        imageTopicSubscriber = null;
                    }
                    if (ros) {
                        ros.close();
                        ros = null;
                    }
                    if (imgElement) {
                        imgElement.src = ''; // Clear current image
                        imgElement.style.display = 'none';
                    }

                    // Reset performance counters
                    isProcessingFrame = false;
                    lastFrameTime = 0;
                    frameCount = 0;
                    startTime = Date.now();
                    
                    if (snapshotButton) {
                        snapshotButton.style.display = 'none';
                    }


                    if (!rosbridgeUrl || !rosImageTopic) {
                        displayMessage('ZED Camera: ROSBridge URL or Image Topic not configured.', 'warning');
                        return;
                    }

                    console.log(`ZED Plugin: Connecting to ROSBridge at: ${rosbridgeUrl}`);
                    console.log(`ZED Plugin: Subscribing to topic: ${rosImageTopic} with throttle: ${throttleRate}ms`);

                    ros = new ROSLIB.Ros({
                        url: rosbridgeUrl
                    });

                    ros.on('connection', () => {
                        console.log('ZED Plugin: Connected to ROSBridge.');
                        logTiming('ROS Connection Established');
                        displayMessage('Connected to ZED Camera. Waiting for image stream...', 'info');

                        imageTopicSubscriber = new ROSLIB.Topic({
                            ros: ros,
                            name: rosImageTopic,
                            messageType: 'sensor_msgs/CompressedImage',
                            throttle_rate: throttleRate
                        });

                        imageTopicSubscriber.subscribe((message) => {
                            const frameStartTime = logTiming('Frame Received');

                            // Frame dropping - skip if we're still processing the last frame
                            if (isProcessingFrame) {
                                console.log('ZED Plugin: Dropping frame - still processing previous frame');
                                return;
                            }

                            // Additional frame rate limiting - ensure minimum time between frames
                            const timeSinceLastFrame = frameStartTime - lastFrameTime;
                            if (timeSinceLastFrame < throttleRate * 0.8) { // 80% of throttle rate as buffer
                                console.log(`ZED Plugin: Dropping frame - too soon (${timeSinceLastFrame}ms < ${throttleRate * 0.8}ms)`);
                                return;
                            }

                            isProcessingFrame = true;
                            frameCount++;
                            lastFrameTime = frameStartTime;

                            // Log frame rate every 10 frames
                            if (frameCount % 10 === 0) {
                                const avgFps = (frameCount * 1000) / (frameStartTime - startTime);
                                console.log(`ZED Plugin: Processed ${frameCount} frames, avg fps: ${avgFps.toFixed(2)}`);
                            }

                            try {
                                let imageData = message.data;

                                if (typeof imageData === 'string') {
                                    logTiming('String Processing Start');

                                    const dataUrl = `data:image/jpeg;base64,${imageData}`;

                                    logTiming('Data URL Created');

                                    if (imgElement) {
                                        imgElement.src = dataUrl;
                                        imgElement.style.display = 'block';
                                        if (snapshotButton) {
                                            snapshotButton.style.display = 'block';
                                        }
                                    }

                                    logTiming('Image Element Updated');

                                    if (errorMessageElement && errorMessageElement.parentElement) {
                                        errorMessageElement.parentElement.removeChild(errorMessageElement);
                                        errorMessageElement = null;
                                    }

                                } else if (imageData instanceof ArrayBuffer || imageData instanceof Uint8Array) {
                                    logTiming('Binary Processing Start');

                                    const bytes = new Uint8Array(imageData);
                                    let binaryString = '';
                                    const chunkSize = 1024;

                                    for (let i = 0; i < bytes.length; i += chunkSize) {
                                        const chunk = bytes.slice(i, i + chunkSize);
                                        binaryString += String.fromCharCode.apply(null, chunk);
                                    }

                                    const base64 = btoa(binaryString);
                                    const dataUrl = `data:image/jpeg;base64,${base64}`;

                                    logTiming('Binary to Base64 Conversion Complete');

                                    if (imgElement) {
                                        imgElement.src = dataUrl;
                                        imgElement.style.display = 'block';
                                        if (snapshotButton) {
                                            snapshotButton.style.display = 'block';
                                        }
                                    }

                                    logTiming('Binary Image Element Updated');

                                    if (errorMessageElement && errorMessageElement.parentElement) {
                                        errorMessageElement.parentElement.removeChild(errorMessageElement);
                                        errorMessageElement = null;
                                    }

                                } else {
                                    console.error('ZED Plugin: Unexpected image data format:', typeof imageData);
                                    displayMessage('ZED Camera: Unexpected image data format from ROSBridge.', 'error');
                                }

                            } catch (error) {
                                console.error('ZED Plugin: Error processing frame:', error);
                                displayMessage('ZED Camera: Error processing image frame.', 'error');
                            } finally {
                                isProcessingFrame = false;
                                logTiming('Frame Processing Complete');
                            }
                        });
                    });

                    ros.on('error', (error) => {
                        console.error('ZED Plugin: ROSBridge error:', error);
                        displayMessage('ZED Camera: ROSBridge connection error. Check URL and server.', 'error');
                        isProcessingFrame = false;
                        if (imageTopicSubscriber) {
                            imageTopicSubscriber.unsubscribe();
                            imageTopicSubscriber = null;
                        }
                    });

                    ros.on('close', (event) => {
                        console.log('ZED Plugin: ROSBridge closed:', event.code, event.reason);
                        isProcessingFrame = false;

                        if (!event.wasClean) {
                            displayMessage('ZED Camera: ROSBridge disconnected unexpectedly. Attempting to reconnect...', 'error');
                            setTimeout(() => {
                                connectAndSubscribe(domainObject.rosbridgeUrl, domainObject.rosImageTopic, domainObject.throttleRate);
                            }, 5000);
                        } else {
                            displayMessage('ZED Camera: Disconnected.', 'info');
                        }

                        if (imageTopicSubscriber) {
                            imageTopicSubscriber.unsubscribe();
                            imageTopicSubscriber = null;
                        }
                        if (imgElement) {
                            imgElement.src = '';
                            imgElement.style.display = 'none';
                        }
                        if (snapshotButton) {
                             snapshotButton.style.display = 'none';
                        }
                    });
                };

                const takeSnapshot = () => {
                    if (imgElement && imgElement.src) {
                        try {
                            // Create a temporary link element to trigger the download
                            const link = document.createElement('a');
                            link.href = imgElement.src;
                            
                            // Generate a filename
                            const now = new Date();
                            const timestamp = now.toISOString().replace(/[:.]/g, '-');
                            const filename = `zed-camera-snapshot-${timestamp}.jpeg`;
                            
                            link.download = filename;
                            document.body.appendChild(link);
                            link.click();
                            document.body.removeChild(link);
                            
                            console.log(`Screenshot saved: ${filename}`);
                            openmct.notifications.info('Snapshot captured successfully!');
                        } catch (error) {
                            console.error('Error taking screenshot:', error);
                            openmct.notifications.error('Snapshot failed: ' + error.message);
                        }
                    } else {
                        console.warn('Cannot take screenshot: No image data available.');
                        openmct.notifications.error('Snapshot failed: No image data available.');
                    }
                };

                return {
                    show(element) {
                        viewContainerElement = element;
                        
                        const container = document.createElement('div');
                        container.style.width = '100%';
                        container.style.height = '100%';
                        container.style.position = 'relative';

                        imgElement = document.createElement('img');
                        imgElement.style.width = '100%';
                        imgElement.style.height = '100%';
                        imgElement.style.objectFit = 'contain';
                        imgElement.style.display = 'none';

                        imgElement.loading = 'eager';
                        imgElement.decoding = 'async';
                        
                        container.appendChild(imgElement);

                        snapshotButton = document.createElement('button');
                        snapshotButton.style.position = 'absolute';
                        snapshotButton.style.bottom = '15px';
                        snapshotButton.style.left = '50%';
                        snapshotButton.style.transform = 'translateX(-50%)';
                        snapshotButton.style.width = '60px';
                        snapshotButton.style.height = '60px';
                        snapshotButton.style.backgroundColor = 'transparent';
                        snapshotButton.style.border = '2px solid white';
                        snapshotButton.style.borderRadius = '50%';
                        snapshotButton.style.cursor = 'pointer';
                        snapshotButton.style.display = 'none';
                        snapshotButton.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)';
                        snapshotButton.style.outline = 'none';
                        
                        snapshotButton.addEventListener('mousedown', () => {
                            snapshotButton.style.transform = 'translateX(-50%) scale(0.95)';
                            snapshotButton.style.boxShadow = '0 2px 4px rgba(0,0,0,0.2)';
                        });
                        snapshotButton.addEventListener('mouseup', () => {
                            snapshotButton.style.transform = 'translateX(-50%) scale(1)';
                            snapshotButton.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)';
                        });
                        snapshotButton.addEventListener('mouseleave', () => {
                            snapshotButton.style.transform = 'translateX(-50%) scale(1)';
                            snapshotButton.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)';
                        });

                        snapshotButton.addEventListener('click', takeSnapshot);
                        container.appendChild(snapshotButton);

                        innerCircle = document.createElement('div');
                        innerCircle.style.width = '45px';
                        innerCircle.style.height = '45px';
                        innerCircle.style.backgroundColor = 'white';
                        innerCircle.style.border = 'none';
                        innerCircle.style.borderRadius = '50%';
                        innerCircle.style.position = 'absolute';
                        innerCircle.style.top = '50%';
                        innerCircle.style.left = '50%';
                        innerCircle.style.transform = 'translate(-50%, -50%)';
                        innerCircle.style.boxSizing = 'border-box';
                        snapshotButton.appendChild(innerCircle);
                        
                        element.appendChild(container);

                        logTiming('View Shown - Starting Connection');
                        connectAndSubscribe(domainObject.rosbridgeUrl, domainObject.rosImageTopic, domainObject.throttleRate);
                    },

                    onEditModeChange(editMode) {
                        if (editMode) {
                            if (imageTopicSubscriber) {
                                imageTopicSubscriber.unsubscribe();
                                imageTopicSubscriber = null;
                            }
                            if (ros) {
                                ros.close();
                                ros = null;
                            }
                            isProcessingFrame = false;
                            if (imgElement) {
                                imgElement.src = '';
                                imgElement.style.display = 'none';
                            }
                            if (snapshotButton) {
                                snapshotButton.style.display = 'none';
                            }
                            displayMessage('ZED Camera: In edit mode. Stream paused.', 'info');
                        } else {
                            logTiming('Edit Mode Ended - Reconnecting');
                            connectAndSubscribe(domainObject.rosbridgeUrl, domainObject.rosImageTopic, domainObject.throttleRate);
                        }
                    },

                    destroy: function () {
                        console.log('ZED Plugin: Destroying view...');
                        
                        if (snapshotButton) {
                            snapshotButton.removeEventListener('click', takeSnapshot);
                            snapshotButton.removeEventListener('mousedown', () => {});
                            snapshotButton.removeEventListener('mouseup', () => {});
                            snapshotButton.removeEventListener('mouseleave', () => {});
                            if (snapshotButton.parentElement) {
                                snapshotButton.parentElement.removeChild(snapshotButton);
                            }
                            snapshotButton = null;
                        }
                        if (innerCircle && innerCircle.parentElement) {
                            innerCircle.parentElement.removeChild(innerCircle);
                            innerCircle = null;
                        }

                        if (imageTopicSubscriber) {
                            imageTopicSubscriber.unsubscribe();
                            imageTopicSubscriber = null;
                        }
                        if (ros) {
                            ros.close();
                            ros = null;
                        }
                        if (imgElement && imgElement.parentElement) {
                            imgElement.parentElement.removeChild(imgElement);
                        }
                        imgElement = null;
                        if (errorMessageElement && errorMessageElement.parentElement) {
                            errorMessageElement.parentElement.removeChild(errorMessageElement);
                        }
                        errorMessageElement = null;
                        viewContainerElement = null;
                        isProcessingFrame = false;

                        console.log('ZED Camera View destroyed.');
                    }
                };
            }
        });

        return {
            destroy: () => {}
        };
    };
};