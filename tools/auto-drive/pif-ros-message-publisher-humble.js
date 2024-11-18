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
            leftCameraInfoPub: null,
            rightCameraInfoPub: null,
            stereoTransformPub: null, // Transform publisher
            tfPub: null, // Transform publisher
            baseline: 0.065, // 6.5 cm
            frameCount: 0, // フレームカウント
            frameIntervalSec: 1 / 30, // フレーム間隔 (30 FPS)
            startTimestamp: null, // 初期タイムスタンプ
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
        this.vslam.leftCameraInfoPub = node.createPublisher('sensor_msgs/msg/CameraInfo', '/camera_info_left');
        this.vslam.rightCameraInfoPub = node.createPublisher('sensor_msgs/msg/CameraInfo', '/camera_info_right');
        this.vslam.stereoTransformPub = node.createPublisher('geometry_msgs/msg/TransformStamped', '/stereo_camera/stereo_transform');
        this.vslam.tfPub = node.createPublisher('tf2_msgs/msg/TFMessage', '/tf');

        // Initialize subscribers
        node.createSubscription('nav_msgs/msg/Odometry', '/odometry/filtered', (data) => {
            for (const callback of this.odometry_callbacks) {
                callback(this.convertOdometryToObj(data));
            }
        });

        this.vslam.startTimestamp = this.node.getClock().now();

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

    createCameraInfo(width, height, fov, baseline = 0) {
        const fx = (width / 2) / Math.tan((fov * Math.PI / 180) / 2);
        const fy = fx;
        const cx = width / 2;
        const cy = height / 2;

        return {
            header: {
                frame_id: 'stereo_camera_frame',
            },
            width: width,
            height: height,
            distortion_model: 'plumb_bob',
            d: [0.0, 0.0, 0.0, 0.0, 0.0], // No distortion
            k: [fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0], // Intrinsic matrix
            r: [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0], // Rectification matrix
            p: [fx, 0.0, cx, -fx * baseline, // tx = -fx * baseline (右カメラ用)
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0], // Projection matrix
            binning_x: 1,
            binning_y: 1,
            roi: {
                x_offset: 0,
                y_offset: 0,
                height: 0,
                width: 0,
                do_rectify: false,
            },
        };
    }
    createStereoTransform(timestampSec) {
        // 右カメラへの基線距離を持つ変換を作成
        return {
            header: {
                stamp: timestampSec,
                frame_id: 'stereo_camera_left', // 左カメラを基準
            },
            child_frame_id: 'stereo_camera_right', // 右カメラの座標系
            transform: {
                translation: {
                    x: this.vslam.baseline, // X方向に基線距離分だけオフセット
                    y: 0.0,
                    z: 0.0,
                },
                rotation: {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0, // 回転なし (単位クォータニオン)
                },
            },
        };
    }
    createTfTransform(parentFrame, childFrame, translation, rotation, timestampSec) {
        return {
            header: {
                stamp: timestampSec,
                frame_id: parentFrame,
            },
            child_frame_id: childFrame,
            transform: {
                translation: {
                    x: translation.x,
                    y: translation.y,
                    z: translation.z,
                },
                rotation: {
                    x: rotation.x,
                    y: rotation.y,
                    z: rotation.z,
                    w: rotation.w,
                },
            },
        };
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

            //camera image
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

            const { seconds, nanoseconds } = (() => {
                const { seconds, nanoseconds } = this.vslam.startTimestamp.secondsAndNanoseconds;
                const additionalNanoseconds = Math.floor(this.vslam.frameCount * this.vslam.frameIntervalSec * 1e9);
                const totalNanoseconds = nanoseconds + additionalNanoseconds;
                return {
                    seconds: seconds + Math.floor(totalNanoseconds / 1e9),
                    nanoseconds: totalNanoseconds % 1e9,
                };
            })();

            const rosTimestamp = { sec: seconds, nanosec: nanoseconds };
            //console.log(seconds, String(nanoseconds).padStart(9, '0'));
            //const rosTimestamp = this.node.getClock().now();

            // フレームカウントをインクリメント
            this.vslam.frameCount++;

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

            //camera info
            const leftCameraInfo = this.createCameraInfo(halfWidth, height, 90, 0);
            const rightCameraInfo = this.createCameraInfo(halfWidth, height, 90, -this.vslam.baseline);

            leftCameraInfo.header.stamp = rosTimestamp;
            rightCameraInfo.header.stamp = rosTimestamp;

            this.vslam.leftCameraInfoPub.publish(leftCameraInfo);
            this.vslam.rightCameraInfoPub.publish(rightCameraInfo);

            // Stereo Transformを生成してパブリッシュ
            //const stereoTransform = this.createStereoTransform(rosTimestamp);
            //this.vslam.stereoTransformPub.publish(stereoTransform);

            // Transform data for /tf
            const tfMessage = {
                transforms: [
                    // Base to Left Camera
                    this.createTfTransform(
                        'base_link',
                        'stereo_camera_left',
                        { x: 0.1, y: 0.0, z: 0.5 },
                        { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                        rosTimestamp
                    ),
                    // Left Camera to Right Camera
                    this.createTfTransform(
                        'stereo_camera_left',
                        'stereo_camera_right',
                        { x: this.vslam.baseline, y: 0.0, z: 0.0 },
                        { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                        rosTimestamp
                    ),
                ],
            };
            
            this.vslam.tfPub.publish(tfMessage);

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
