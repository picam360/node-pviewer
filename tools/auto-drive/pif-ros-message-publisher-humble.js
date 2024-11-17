const rclnodejs = require('rclnodejs');
const nmeaParse = require('nmea-simple');
const utm = require('utm');
const Quaternion = require('quaternion');
const cv = require('opencv4nodejs');

class PifRosMessagePublisher {
    constructor() {
        this.isInitialized = false;

        this.gps = {
            initialLat: null,
            initialLon: null,
            currentX: 0,
            currentY: 0,
            path: {
                header: { frame_id: 'odom' },
                poses: [],
            },
            nmeaData: null,
            odomPub: null,
            pathPub: null,
        };

        this.encoder = {
            left_counts: 0,
            right_counts: 0,
            offset_left_counts: null,
            offset_right_counts: null,
            wheel_separation: 0.11,
            wheel_radius: 0.03,
            ticks_per_revolution: 1920,
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            last_time: null,
            path: {
                header: { frame_id: 'odom' },
                poses: [],
            },
            odomPub: null,
            posePub: null,
            pathPub: null,
        };

        this.vslam = {
            leftImagePub: null,
            rightImagePub: null,
        };

        this.odometry_callbacks = [];
    }

    async initialize() {
        if (this.isInitialized) {
            return;
        }

        await rclnodejs.init();
        const node = rclnodejs.createNode('pif_ros_message_publisher');
        this.node = node;

        console.log('pif_ros_message_publisher initialized successfully');

        // Initialize publishers
        this.encoder.odomPub = node.createPublisher('nav_msgs/msg/Odometry', '/wheel/odometry');
        this.encoder.posePub = node.createPublisher('geometry_msgs/msg/PoseStamped', '/wheel/pose');
        this.encoder.pathPub = node.createPublisher('nav_msgs/msg/Path', '/wheel/path');
        this.gps.odomPub = node.createPublisher('nav_msgs/msg/Odometry', '/gps/odometry');
        this.gps.pathPub = node.createPublisher('nav_msgs/msg/Path', '/gps/path');
        this.vslam.leftImagePub = node.createPublisher('sensor_msgs/msg/Image', '/stereo_camera/left/image');
        this.vslam.rightImagePub = node.createPublisher('sensor_msgs/msg/Image', '/stereo_camera/right/image');

        // Initialize subscribers
        node.createSubscription('nav_msgs/msg/Odometry', '/odometry/filtered', (data) => {
            for (const callback of this.odometry_callbacks) {
                callback(this.convertOdometryToObj(data));
            }
        });

        rclnodejs.spin(node);
        this.isInitialized = true;
    }

    publishGpsNmea(nmea_str, timestamp_sec) {
        try {
            const nmea = nmeaParse.parseNmeaSentence(nmea_str);
            if (!nmea || nmea.sentenceId !== 'GGA') {
                return;
            }

            const { easting, northing } = utm.fromLatLon(nmea.latitude, nmea.longitude);
            if (this.gps.initialLat === null) {
                const { easting: initialEasting, northing: initialNorthing } = utm.fromLatLon(nmea.latitude, nmea.longitude);
                this.gps.initialLat = nmea.latitude;
                this.gps.initialLon = nmea.longitude;
                this.gps.currentX = easting - initialEasting;
                this.gps.currentY = northing - initialNorthing;
            }

            const odom = {
                header: {
                    stamp: this.node.getClock().now(),
                    frame_id: 'odom',
                },
                child_frame_id: 'base_link',
                pose: {
                    pose: {
                        position: {
                            x: this.gps.currentX,
                            y: this.gps.currentY,
                            z: 0.0,
                        },
                        orientation: { x: 0, y: 0, z: 0, w: 1 },
                    },
                    covariance: Array(36).fill(0.0001),
                },
                twist: {
                    twist: {
                        linear: { x: 0, y: 0, z: 0 },
                        angular: { x: 0, y: 0, z: 0 },
                    },
                    covariance: Array(36).fill(0.0001),
                },
            };

            this.gps.odomPub.publish(odom);

            const poseStamped = {
                header: odom.header,
                pose: odom.pose.pose,
            };
            this.gps.path.poses.push(poseStamped);
            this.gps.pathPub.publish(this.gps.path);
        } catch (err) {
            console.error('Error publishing GPS NMEA:', err);
        }
    }
    
    publishWheelCount(data, timestamp_sec) {
        try {
            const rosTimestamp = this.node.getClock().now();
    
            this.encoder.left_counts = Math.floor(data[0]);
            this.encoder.right_counts = Math.floor(data[1]);
    
            if (this.encoder.offset_left_counts === null) {
                this.encoder.offset_left_counts = this.encoder.left_counts;
                this.encoder.offset_right_counts = this.encoder.right_counts;
                this.encoder.last_time = timestamp_sec;
            }
    
            this.encoder.left_counts -= this.encoder.offset_left_counts;
            this.encoder.right_counts -= this.encoder.offset_right_counts;
    
            let dt = timestamp_sec - this.encoder.last_time;
    
            if (dt === 0 || this.encoder.offset_left_counts === null) {
                return;
            }
    
            let delta_left = this.encoder.left_counts - this.encoder.last_left_counts;
            let delta_right = this.encoder.right_counts - this.encoder.last_right_counts;
    
            let distance_left = 2 * Math.PI * this.encoder.wheel_radius * delta_left / this.encoder.ticks_per_revolution;
            let distance_right = 2 * Math.PI * this.encoder.wheel_radius * delta_right / this.encoder.ticks_per_revolution;
    
            let distance = (distance_left + distance_right) / 2;
            let dtheta = (distance_right - distance_left) / this.encoder.wheel_separation;
    
            let dx, dy;
            if (Math.abs(dtheta) < 1e-6) {
                dx = distance * Math.cos(this.encoder.theta);
                dy = distance * Math.sin(this.encoder.theta);
            } else {
                let radius = distance / dtheta;
                dx = radius * (Math.sin(this.encoder.theta + dtheta) - Math.sin(this.encoder.theta));
                dy = radius * (Math.cos(this.encoder.theta) - Math.cos(this.encoder.theta + dtheta));
            }
    
            this.encoder.x += dx;
            this.encoder.y += dy;
            this.encoder.theta += dtheta;
    
            let odom = {
                header: {
                    stamp: rosTimestamp,
                    frame_id: "odom",
                },
                child_frame_id: "base_link",
                pose: {
                    pose: {
                        position: {
                            x: this.encoder.x,
                            y: this.encoder.y,
                            z: 0.0,
                        },
                        orientation: Quaternion.fromEuler(0, 0, this.encoder.theta),
                    },
                    covariance: Array(36).fill(0.01), // Adjust as needed
                },
                twist: {
                    twist: {
                        linear: { x: distance / dt, y: 0, z: 0 },
                        angular: { x: 0, y: 0, z: dtheta / dt },
                    },
                    covariance: Array(36).fill(0.01), // Adjust as needed
                },
            };
    
            this.encoder.odomPub.publish(odom);
    
            let pose = {
                header: odom.header,
                pose: odom.pose.pose,
            };
    
            this.encoder.posePub.publish(pose);
    
            this.encoder.path.header.stamp = rosTimestamp;
            this.encoder.path.poses.push(pose);
            this.encoder.pathPub.publish(this.encoder.path);
    
            this.encoder.last_left_counts = this.encoder.left_counts;
            this.encoder.last_right_counts = this.encoder.right_counts;
            this.encoder.last_time = timestamp_sec;
        } catch (err) {
            console.error("Error in publishWheelCount:", err);
        }
    }

    /**
     * Publish side-by-side stereo images to /stereo_camera/left/image and /stereo_camera/right/image
     * @param {Buffer} sideBySideImageBuffer - Buffer containing the side-by-side image.
     * @param {number} timestampSec - Timestamp in seconds.
     */
    async publishVslam(sideBySideImageBuffer, timestampSec) {
        try {
            const image = cv.imdecode(sideBySideImageBuffer);

            const { cols: width, rows: height } = image;

            const halfWidth = Math.floor(width / 2);

            const leftROI = new cv.Rect(0, 0, halfWidth, height);
            if (leftROI.width <= 0 || leftROI.height <= 0) {
                throw new Error('Invalid left ROI dimensions.');
            }
            const leftImage = image.getRegion(leftROI).copy();

            const rightROI = new cv.Rect(halfWidth, 0, halfWidth, height);
            if (rightROI.width <= 0 || rightROI.height <= 0) {
                throw new Error('Invalid right ROI dimensions.');
            }
            const rightImage = image.getRegion(rightROI).copy();

            const leftImageData = {
                height: height,
                width: halfWidth,
                encoding: 'bgr8',
                is_bigendian: 0,
                step: halfWidth * 3,
                data: Array.from(leftImage.getData()),
            };

            const rightImageData = {
                height: height,
                width: halfWidth,
                encoding: 'bgr8',
                is_bigendian: 0,
                step: halfWidth * 3,
                data: Array.from(rightImage.getData()),
            };

            const rosTimestamp = this.node.getClock().now();

            const leftImageMsg = {
                header: {
                    stamp: rosTimestamp,
                    frame_id: 'stereo_camera_left',
                },
                height: leftImageData.height,
                width: leftImageData.width,
                encoding: leftImageData.encoding,
                is_bigendian: leftImageData.is_bigendian,
                step: leftImageData.step,
                data: leftImageData.data,
            };
            this.vslam.leftImagePub.publish(leftImageMsg);

            const rightImageMsg = {
                header: {
                    stamp: rosTimestamp,
                    frame_id: 'stereo_camera_right',
                },
                height: rightImageData.height,
                width: rightImageData.width,
                encoding: rightImageData.encoding,
                is_bigendian: rightImageData.is_bigendian,
                step: rightImageData.step,
                data: rightImageData.data,
            };
            this.vslam.rightImagePub.publish(rightImageMsg);

            console.log('Published VSLAM images.');
        } catch (err) {
            console.error('Error in publishVslam:', err);
        }
    }

    subscribeOdometry(callback) {
        this.odometry_callbacks.push(callback);
    }

    convertOdometryToObj(data) {
        return {
            header: data.header,
            pose: data.pose.pose,
            twist: data.twist.twist,
        };
    }
}

module.exports = PifRosMessagePublisher;
