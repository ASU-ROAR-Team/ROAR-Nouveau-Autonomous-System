// src/plugins/Arm-Control/plugin.js

// This plugin assumes that window.ArmControlView is available globally,
// which means ArmControlView.js must be loaded BEFORE this script in index.html.
// It also assumes openmct is available globally.

(function () {
    const ARM_CONTROL_KEY = 'arm-control';
    const ARM_ROOT_KEY = 'arm-root';
    const MAIN_ARM_INSTANCE_KEY = 'main-arm-sliders'; // Renamed key to reflect new UI

    function ArmControlPlugin() {
        return function install(openmct) {
            console.log('Arm Control Plugin: install function started.');

            // 1. Define a new object type for your arm control
            openmct.types.addType(ARM_CONTROL_KEY, {
                name: 'Robotic Arm Control Sliders', // Updated name to reflect new UI
                description: 'Slider-based velocity control for a 6-DOF robotic arm via ROS joint velocity messages.',
                cssClass: 'icon-telemetry',
                creatable: true,
                def: {
                    type: ARM_CONTROL_KEY
                },
                initialize: function (domainObject) {
                    domainObject.name = domainObject.name || 'New Arm Control';
                },
                form: []
            });

            // 2. Add Object Provider - This resolves objects by their identifiers
            openmct.objects.addProvider(ARM_CONTROL_KEY, {
                get: function (identifier) {
                    console.log('Object Provider: Getting object for identifier ->', identifier);

                    if (identifier.key === ARM_ROOT_KEY) {
                        // Return the "Arm Controls" folder object
                        return Promise.resolve({
                            identifier: identifier,
                            name: 'Arm Controls',
                            type: 'folder',
                            location: 'ROOT'
                        });
                    } else if (identifier.key === MAIN_ARM_INSTANCE_KEY) {
                        // Return the main arm slider instance object
                        return Promise.resolve({
                            identifier: identifier,
                            name: 'Main Robotic Arm Sliders', // Updated name to reflect new UI
                            type: ARM_CONTROL_KEY,
                            location: `${ARM_CONTROL_KEY}:${ARM_ROOT_KEY}`
                        });
                    }

                    return Promise.reject(new Error('Unknown object: ' + identifier.key));
                }
            });

            // 3. Add Arm Controls as a ROOT object
            openmct.objects.addRoot({
                namespace: ARM_CONTROL_KEY,
                key: ARM_ROOT_KEY
            });

            // 4. COMPOSITION PROVIDER (Contents *of* your "Arm Controls" folder)
            openmct.composition.addProvider({
                appliesTo: function (domainObject) {
                    const matches = domainObject.identifier.namespace === ARM_CONTROL_KEY &&
                                    domainObject.identifier.key === ARM_ROOT_KEY;
                    console.log('COMPOSITION PROVIDER (Arm Folder): Identifier ->', domainObject.identifier, '| Matches ->', matches);
                    return matches;
                },
                load: function (domainObject) {
                    console.log('Loading children for Arm Controls folder. Adding arm instance.');
                    return Promise.resolve([
                        {
                            namespace: ARM_CONTROL_KEY,
                            key: MAIN_ARM_INSTANCE_KEY
                        }
                    ]);
                }
            });

            // 5. Register the view provider for your 'arm-control' type
            openmct.objectViews.addProvider({
                key: 'arm-control-view',
                name: 'Arm Control Sliders', // Updated name to reflect new UI
                cssClass: 'icon-telemetry',
                canView: function (domainObject) {
                    return domainObject.type === ARM_CONTROL_KEY;
                },
                view: function (domainObject) {
                    let armControlInstance = null;
                    return {
                        show: function (element) {
                            if (typeof window.ArmControlView === 'undefined') {
                                console.error('ArmControlView not found. Make sure ArmControlView.js is loaded before plugin.js.');
                                element.innerHTML = '<p style="color: red;">Error: ArmControlView component not loaded.</p>';
                                return;
                            }
                            armControlInstance = new window.ArmControlView(element, openmct);
                            armControlInstance.render();
                        },
                        destroy: function (element) {
                            if (armControlInstance) {
                                armControlInstance.destroy();
                                armControlInstance = null;
                            }
                        }
                    };
                }
            });

            console.log('Arm Control Plugin installed successfully.');
        };
    }

    // Expose ArmControlPlugin globally
    window.ArmControlPlugin = ArmControlPlugin;

})();
