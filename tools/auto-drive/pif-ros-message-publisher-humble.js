const rclnodejs = require('rclnodejs');
const nmeaParse = require('nmea-simple');
const utm = require('utm');
const Quaternion = require('quaternion');
const cv = require('opencv4nodejs');

class PifRosMessagePublisher {
    constructor(is_mono = true) {
        this.isInitialized = false;

        this.is_mono = is_mono;
        this.vslam_scale = 0.5;
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
            left_direction: 1,
            right_direction: -1,
            last_left_counts: null,
            last_right_counts: null,
            meter_per_pulse: 0.007512237883246996,
            //wheel_separation: 0.11,
            wheel_separation: 2.5785047010989204,
            imu_heading_error: 19.48381320843465,
            x: 0.0,
            y: 0.0,
            heading: 0.0,
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
            tfPub: null, // Transform publisher
            tfStaticPub: null, // Transform publisher
            imuPub: null, // imu publisher
            baseline: 0.065, // parallax
            frameCount: 0, // フレームカウント
            frameIntervalSec: 1 / 30, // フレーム間隔 (30 FPS)
            startTimestamp: null, // 初期タイムスタンプ
        };

        this.odometry_callbacks = [];
    }

    async initialize(params) {
        if (this.isInitialized) {
            return;
        }

        params = params || {};

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
        if(this.is_mono){
            this.vslam.leftImagePub = node.createPublisher('sensor_msgs/msg/Image', '/visual_slam/image_0');
            this.vslam.rightImagePub = node.createPublisher('sensor_msgs/msg/Image', '/visual_slam/image_1');
        }else{
            this.vslam.leftImagePub = node.createPublisher('sensor_msgs/msg/Image', '/left/image_rect');
            this.vslam.rightImagePub = node.createPublisher('sensor_msgs/msg/Image', '/right/image_rect');
        }
        if(this.is_mono){
            this.vslam.leftCameraInfoPub = node.createPublisher('sensor_msgs/msg/CameraInfo', '/visual_slam/camera_info_0');
            this.vslam.rightCameraInfoPub = node.createPublisher('sensor_msgs/msg/CameraInfo', '/visual_slam/camera_info_1');
        }else{
            this.vslam.leftCameraInfoPub = node.createPublisher('sensor_msgs/msg/CameraInfo', '/left/camera_info_rect');
            this.vslam.rightCameraInfoPub = node.createPublisher('sensor_msgs/msg/CameraInfo', '/right/camera_info_rect');
        }
        this.vslam.tfPub = node.createPublisher('tf2_msgs/msg/TFMessage', '/tf');

        const qosProfile = new rclnodejs.QoS(
            rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,  // 履歴のポリシー
            10,  // キューの深さ
            rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,  // 信頼性ポリシー
            rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,  // メッセージの耐久性
        );
        this.vslam.tfStaticPub = node.createPublisher('tf2_msgs/msg/TFMessage', '/tf_static', {qos : qosProfile});
        this.vslam.imuPub = node.createPublisher('sensor_msgs/msg/Imu', '/visual_slam/imu');

        // Initialize subscribers
        node.createSubscription('nav_msgs/msg/Odometry', '/visual_slam/tracking/odometry', (data) => {
            for (const callback of this.odometry_callbacks) {
                callback(this.convertOdometryToObj(data));
            }
        });

        rclnodejs.spin(node);

        this.vslam.startTimestamp = this.node.getClock().now();

        const bearing_orientation = Quaternion.fromEuler((90 - (params.heading || 0)) * Math.PI / 180, 0, 0, "ZXY");
        //const camera_orientation = { x: 0.0, y: 0.0, z: 0.0, w: 1.0 };
        let camera_orientation = Quaternion.fromEuler(-90 * Math.PI / 180, -90 * Math.PI / 180, 0 * Math.PI / 180, "ZXY");
        camera_orientation = bearing_orientation.mul(camera_orientation);
        // Transform data for /tf_static
        const tfStaticMessage = {
            transforms: [
                // Base to Front Camera
                this.createTfTransform(
                    'base_link',
                    'front_stereo_camera',
                    { x: 0.0, y: 0.0, z: 1.0 },
                    //{ x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                    camera_orientation,
                    this.vslam.startTimestamp
                ),
                // Front Camera to Front Left Camera
                this.createTfTransform(
                    'front_stereo_camera',
                    'front_stereo_camera_left_optical',
                    { x: 0.0, y: 0.0, z: 0.0 },
                    { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                    this.vslam.startTimestamp
                ),
                // Front Camera to Front Right Camera
                this.createTfTransform(
                    'front_stereo_camera',
                    'front_stereo_camera_right_optical',
                    { x: 0.0, y: -this.vslam.baseline, z: 0.0 },
                    { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                    this.vslam.startTimestamp
                ),
                // Base to imu
                this.createTfTransform(
                    'base_link',
                    'imu_frame',
                    { x: 0.0, y: 0.0, z: 1.0 },
                    { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                    this.vslam.startTimestamp
                ),
            ],
        };
        
        this.vslam.tfStaticPub.publish(tfStaticMessage);

        this.isInitialized = true;
    }

    publishGpsNmea(nmea_str, rosTimestamp) {
        try {
            const nmea = nmeaParse.parseNmeaSentence(nmea_str);
            if (!nmea || nmea.sentenceId !== 'GGA') {
                return;
            }

            const { easting, northing } = utm.fromLatLon(nmea.latitude, nmea.longitude);
            if (this.gps.initialLat === null) {
                this.gps.initialLat = nmea.latitude;
                this.gps.initialLon = nmea.longitude;
            }
            const { easting: initialEasting, northing: initialNorthing } = utm.fromLatLon(this.gps.initialLat, this.gps.initialLon);
            this.gps.currentX = easting - initialEasting;
            this.gps.currentY = northing - initialNorthing;

            const odom = {
                header: {
                    stamp: rosTimestamp,
                    frame_id: 'odom',
                },
                child_frame_id: 'base_link',
                pose: {
                    pose: {
                        position: {
                            z: this.gps.currentX * this.vslam_scale,
                            x: -this.gps.currentY * this.vslam_scale,
                            y: 0.0,
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
    
    publishWheelCount(data, heading, rosTimestamp) {
        try {
            this.encoder.left_counts = Math.floor(data[0]) * this.encoder.left_direction;
            this.encoder.right_counts = Math.floor(data[1]) * this.encoder.right_direction;

            if (this.encoder.last_left_counts === null) {
                this.encoder.last_left_counts = this.encoder.left_counts;
                this.encoder.last_right_counts = this.encoder.right_counts;
                this.encoder.last_time = rosTimestamp;
                this.encoder.heading = heading + this.encoder.imu_heading_error;
            }

            function rostimestamp_diff(time1, time2) {
                const secDiff = time2.sec - time1.sec;
                const nanosecDiff = (time2.nanosec - time1.nanosec) / 1e9;
                return secDiff + nanosecDiff;
            }

            let dt = rostimestamp_diff(rosTimestamp, this.encoder.last_time);
    
            if (dt === 0 || this.encoder.last_left_counts === null) {
                return;
            }
    
            let delta_left = this.encoder.left_counts - this.encoder.last_left_counts;
            let delta_right = this.encoder.right_counts - this.encoder.last_right_counts;
    
            let distance_left = delta_left * this.encoder.meter_per_pulse;
            let distance_right = delta_right * this.encoder.meter_per_pulse;
    
            let distance = (distance_left + distance_right) / 2;
            let dtheta = (distance_right - distance_left) / this.encoder.wheel_separation;

            if (distance === 0 && dtheta === 0) {
                return;
            }
    
            let dx, dy;
            const theta = -this.encoder.heading * Math.PI / 180 + Math.PI / 2;
            if (Math.abs(dtheta) < 1e-6) {
                dx = distance * Math.cos(theta);
                dy = distance * Math.sin(theta);
            } else {
                let radius = distance / dtheta;
                dx = radius * (Math.sin(theta + dtheta) - Math.sin(theta));
                dy = radius * (Math.cos(theta) - Math.cos(theta + dtheta));
            }
    
            this.encoder.x += dx;
            this.encoder.y += dy;
            this.encoder.heading += -dtheta * 180 / Math.PI;

            const gps_distance = Math.sqrt(this.gps.currentX * this.gps.currentX + this.gps.currentY * this.gps.currentY);
            const pulse_distance = Math.sqrt(this.encoder.x * this.encoder.x + this.encoder.y * this.encoder.y) / this.encoder.meter_per_pulse;
            const meter_per_pulse_candidate = gps_distance / pulse_distance;

            console.log(this.encoder.left_counts, this.encoder.right_counts, delta_left,  delta_right, this.encoder.x, this.encoder.y, this.encoder.heading, meter_per_pulse_candidate);
    
            let odom = {
                header: {
                    stamp: rosTimestamp,
                    frame_id: "odom",
                },
                child_frame_id: "base_link",
                pose: {
                    pose: {
                        position: {
                            z: this.encoder.x * this.vslam_scale,
                            x: -this.encoder.y * this.vslam_scale,
                            y: 0.0,
                        },
                        orientation: Quaternion.fromEuler(0, 0, theta),
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
            this.encoder.last_time = rosTimestamp;
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
                //frame_id: 'stereo_camera_frame',
                frame_id: baseline==0 ? 'front_stereo_camera_left_optical' : 'front_stereo_camera_right_optical',
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
    
    createTfTransform(parentFrame, childFrame, translation, rotation, rosTimestamp) {
        return {
            header: {
                stamp: rosTimestamp,
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

    async publishImu(imu, rosTimestamp) {
        try {
            const imuMsg = {
                header: {
                    stamp: rosTimestamp,
                    frame_id: 'imu_frame'
                },
                orientation: imu.quaternion,
                orientation_covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0],
                angular_velocity: {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0
                },
                angular_velocity_covariance: [-1, 0, 0, 0, 0, 0, 0, 0, 0],
                linear_acceleration: {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0
                },
                linear_acceleration_covariance: [-1, 0, 0, 0, 0, 0, 0, 0, 0]
            };
            this.vslam.imuPub.publish(imuMsg);

            console.log('Published IMU data.');
        } catch (err) {
            console.error('Error in publishImu:', err);
        }
    }

    /**
     * Publish side-by-side stereo images to /stereo_camera/left/image and /stereo_camera/right/image
     * @param {Buffer} sideBySideImageBuffer - Buffer containing the side-by-side image.
     * @param {number} timestampSec - Timestamp in seconds.
     */
    async publishVslam(sideBySideImageBuffer, rosTimestamp) {
        try {
            const image = cv.imdecode(sideBySideImageBuffer);

            const { cols: width, rows: height } = image;

            const halfWidth = Math.floor(width / 2);

            //camera image
            const leftROI = new cv.Rect(0, 0, halfWidth, height);
            if (leftROI.width <= 0 || leftROI.height <= 0) {
                throw new Error('Invalid left ROI dimensions.');
            }
            let leftImage = image.getRegion(leftROI).copy();
            if(this.is_mono){
                leftImage = leftImage.bgrToGray();
            }

            const rightROI = new cv.Rect(halfWidth, 0, halfWidth, height);
            if (rightROI.width <= 0 || rightROI.height <= 0) {
                throw new Error('Invalid right ROI dimensions.');
            }
            let rightImage = image.getRegion(rightROI).copy();
            if(this.is_mono){
                rightImage = rightImage.bgrToGray();
            }

            const leftImageData = {
                height: height,
                width: halfWidth,
                encoding: (this.is_mono ? 'mono8' : 'bgr8'),
                is_bigendian: 0,
                step: (this.is_mono ? halfWidth : halfWidth * 3),
                data: Array.from(leftImage.getData()),
            };

            const rightImageData = {
                height: height,
                width: halfWidth,
                encoding: (this.is_mono ? 'mono8' : 'bgr8'),
                is_bigendian: 0,
                step: (this.is_mono ? halfWidth : halfWidth * 3),
                data: Array.from(rightImage.getData()),
            };

            // Transform data for /tf
            const tfMessage = {
                transforms: [
                    // Front Camera to Front Left Camera
                    this.createTfTransform(
                        'front_stereo_camera',
                        'front_stereo_camera_left_optical',
                        { x: 0.0, y: 0.0, z: 0.0 },
                        { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                        rosTimestamp
                    ),
                    // Front Camera to Front Right Camera
                    this.createTfTransform(
                        'front_stereo_camera',
                        'front_stereo_camera_right_optical',
                        { x: 0.0, y: -this.vslam.baseline, z: 0.0 },
                        { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                        rosTimestamp
                    ),
                ],
            };
            
            this.vslam.tfPub.publish(tfMessage);

            // フレームカウントをインクリメント
            this.vslam.frameCount++;

            //camera info
            const leftCameraInfo = this.createCameraInfo(halfWidth, height, 90, 0);
            const rightCameraInfo = this.createCameraInfo(halfWidth, height, 90, this.vslam.baseline);

            leftCameraInfo.header.stamp = rosTimestamp;
            rightCameraInfo.header.stamp = rosTimestamp;

            this.vslam.leftCameraInfoPub.publish(leftCameraInfo);
            this.vslam.rightCameraInfoPub.publish(rightCameraInfo);

            const leftImageMsg = {
                header: {
                    stamp: rosTimestamp,
                    //frame_id: 'stereo_camera_left',
                    frame_id: 'front_stereo_camera_left_optical',
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
                    //frame_id: 'stereo_camera_right',
                    frame_id: 'front_stereo_camera_right_optical',
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
