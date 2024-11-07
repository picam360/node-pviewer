const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const nav_msgs = rosnodejs.require('nav_msgs').msg;
const geometry_msgs = rosnodejs.require('geometry_msgs').msg;
const nmeaParse = require('nmea-simple');
const utm = require('utm');
const Quaternion = require('quaternion');

class PifRosMessagePublisher {
    constructor() {
        this.isInitialized = false;
        this.isInitializing = false;

        this.gps = {
            initialLat: null,         // Initial latitude (starting point)
            initialLon: null,         // Initial longitude (starting point)
            currentX: 0,              // Current X position (in meters)
            currentY: 0,              // Current Y position (in meters)

            // Path data structure
            path: {
                header: { frame_id: "odom" },
                poses: []              // Array of PoseStamped
            },

            nmeaData: null,           // Store the latest NMEA data
            odomPub: null,            // Publisher for odometry
            pathPub: null             // Publisher for path
        };


        // Initialize encoder and ROS related properties inside `this.encoder`
        this.encoder = {
            // Encoder-related parameters
            left_counts: 0,
            right_counts: 0,
            last_left_counts: 0,
            last_right_counts: 0,
            offset_left_counts: null,
            offset_right_counts: null,
            wheel_separation: 0.11,  // meters
            wheel_radius: 0.03,      // meters
            ticks_per_revolution: 1920,

            // Position and orientation
            x: 0.0,                  // Initial x position
            y: 0.0,                  // Initial y position
            theta: 228.77850917199368 * Math.PI / 180.0,  // Initial orientation in radians
            last_time: rosnodejs.Time.now(),
            path: {
                header: { frame_id: "odom" },
                poses: []              // Array of PoseStamped
            },

            // Publishers
            odom_pub: null,
            pose_pub: null,
            path_pub: null
        };

        this.odometry_callbacks = [];
    }

    async initialize() {
        if (this.isInitialized || this.isInitializing) {
            return;
        }
        this.isInitializing = true;

        try {
            await rosnodejs.initNode('/pif_ros_message_publisher', {
                onTheFly: true
            });
            this.isInitializing = false;
            this.isInitialized = true;
            console.log('pif_ros_message_publisher initialized successfully');
        } catch (error) {
            this.isInitializing = false;
            this.isInitialized = false;
            console.error('failed to initialize pif_ros_message_publisher:', error);
            return;
        }

        // Initialize the publishers
        this.encoder.odom_pub = rosnodejs.nh.advertise('/wheel/odometry', nav_msgs.Odometry, { queueSize: 50 });
        this.encoder.pose_pub = rosnodejs.nh.advertise('/wheel/pose', geometry_msgs.PoseStamped, { queueSize: 10 });
        this.encoder.path_pub = rosnodejs.nh.advertise('/wheel/path', nav_msgs.Path, { queueSize: 10 });
        this.gps.odomPub = rosnodejs.nh.advertise('/gps/odometry', nav_msgs.Odometry, { queueSize: 50 });
        this.gps.pathPub = rosnodejs.nh.advertise('/gps/path', nav_msgs.Path, { queueSize: 10 });

        //Create Subsclibers
        this.odometrySub = rosnodejs.nh.subscribe('/odometry/filtered', nav_msgs.Odometry, (data) => {
            for(const callback of this.odometry_callbacks){
                callback(this.convertOdometryToObj(data));
            }
        });
    }

    publishGpsNmea(nmea_str, timestamp_sec) {
        try {
            // Parse the NMEA sentence
            const nmea = nmeaParse.parseNmeaSentence(nmea_str);
            if (!nmea || nmea.sentenceId !== 'GGA') {
                return;
            }

            const rosTimestamp = rosnodejs.Time.dateToRosTime(Math.floor(timestamp_sec * 1000));

            // Initialize the initial position (if not already done)
            if (this.gps.initialLat === null) {
                this.gps.initialLat = nmea.latitude;
                this.gps.initialLon = nmea.longitude;
            }

            // Convert latitude and longitude to UTM coordinates
            const { easting, northing } = utm.fromLatLon(nmea.latitude, nmea.longitude);

            // Calculate position relative to initial position
            const { easting: initialEasting, northing: initialNorthing } = utm.fromLatLon(this.gps.initialLat, this.gps.initialLon);

            // Create the Odometry message
            const odom = new nav_msgs.Odometry();
            odom.header.stamp = rosTimestamp;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            // Set position in the Odometry message
            odom.pose.pose.position.x = easting - initialEasting;
            odom.pose.pose.position.y = northing - initialNorthing;
            odom.pose.pose.position.z = 0.0;

            // Set covariance for position uncertainty (to be adjusted later)
            const gpsNoiseStd = 0.0000001;  // Todo: Adjust with fix type
            odom.pose.covariance = [
                gpsNoiseStd ** 2, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, gpsNoiseStd ** 2, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 99999.0
            ];

            // Set velocity (not used here)
            odom.twist.twist.linear.x = 0.0;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.angular.z = 0.0;

            // Publish the Odometry message
            this.gps.odomPub.publish(odom);

            // Create PoseStamped message and add to the path
            const pose = new geometry_msgs.PoseStamped();
            pose.header = odom.header;
            pose.pose = odom.pose.pose;
            this.gps.path.header.stamp = rosTimestamp;
            this.gps.path.poses.push(pose);

            // Publish the path
            this.gps.pathPub.publish(this.gps.path);
        } catch (e) {
            console.error("Error processing NMEA message:", e);
        }
    }


    publishWheelCount(data, timestamp_sec) {
        try {
            // Extract the timestamp and wheel count data
            const rosTimestamp = rosnodejs.Time.dateToRosTime(Math.floor(timestamp_sec * 1000));

            this.encoder.left_counts = Math.floor(data[0]);
            this.encoder.right_counts = Math.floor(data[1]);

            // Initialize the offsets on the first message
            if (this.encoder.offset_left_counts === null) {
                this.encoder.offset_left_counts = this.encoder.left_counts;
                this.encoder.offset_right_counts = this.encoder.right_counts;
                this.encoder.last_time = timestamp_sec;
            }

            // Subtract the offset from the counts
            this.encoder.left_counts -= this.encoder.offset_left_counts;
            this.encoder.right_counts -= this.encoder.offset_right_counts;

            // Update odometry based on the new counts
            let dt = timestamp_sec - this.encoder.last_time;

            // Avoid division by zero if dt is zero
            if (dt === 0 || this.encoder.offset_left_counts === null) {
                return;
            }

            // Calculate the change in wheel counts since the last update
            let delta_left = this.encoder.left_counts - this.encoder.last_left_counts;
            let delta_right = this.encoder.right_counts - this.encoder.last_right_counts;

            // Compute the distance traveled by each wheel
            let distance_left = 2 * Math.PI * this.encoder.wheel_radius * delta_left / this.encoder.ticks_per_revolution;
            let distance_right = 2 * Math.PI * this.encoder.wheel_radius * delta_right / this.encoder.ticks_per_revolution;

            // Calculate the average distance and the angular change
            let distance = (distance_left + distance_right) / 2;
            let dtheta = (distance_right - distance_left) / this.encoder.wheel_separation;

            // Update the position (x, y) based on the calculated distance and angular change
            let dx, dy;
            if (Math.abs(dtheta) < 1e-6) {
                dx = distance * Math.cos(this.encoder.theta);
                dy = distance * Math.sin(this.encoder.theta);
            } else {
                let radius = distance / dtheta;
                dx = radius * (Math.sin(this.encoder.theta + dtheta) - Math.sin(this.encoder.theta));
                dy = radius * (Math.cos(this.encoder.theta) - Math.cos(this.encoder.theta + dtheta));
            }

            // Update the encoder's position and orientation
            this.encoder.x += dx;
            this.encoder.y += dy;
            this.encoder.theta += dtheta;

            // Create the Odometry message
            let odom = new nav_msgs.Odometry();
            odom.header.stamp = rosTimestamp;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            odom.pose.pose.position.x = this.encoder.x;
            odom.pose.pose.position.y = this.encoder.y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = Quaternion.fromEuler(this.encoder.theta, 0, 0);

            // Calculate linear and angular velocity
            odom.twist.twist.linear.x = distance / dt;
            odom.twist.twist.angular.z = dtheta / dt;

            // Set covariance values for position and velocity uncertainties
            const position_covariance = 0.3 * 0.3;  // 30 cm uncertainty
            const orientation_covariance = 0.1 * 0.1;  // Radians uncertainty
            const velocity_covariance = 0.1 * 0.1;  // m/s uncertainty

            odom.pose.covariance = [
                position_covariance, 0, 0, 0, 0, 0,
                0, position_covariance, 0, 0, 0, 0,
                0, 0, position_covariance, 0, 0, 0,
                0, 0, 0, orientation_covariance, 0, 0,
                0, 0, 0, 0, orientation_covariance, 0,
                0, 0, 0, 0, 0, orientation_covariance
            ];

            odom.twist.covariance = [
                velocity_covariance, 0, 0, 0, 0, 0,
                0, velocity_covariance, 0, 0, 0, 0,
                0, 0, velocity_covariance, 0, 0, 0,
                0, 0, 0, velocity_covariance, 0, 0,
                0, 0, 0, 0, velocity_covariance, 0,
                0, 0, 0, 0, 0, velocity_covariance
            ];

            // Publish the Odometry message
            this.encoder.odom_pub.publish(odom);

            // Publish the current pose as PoseStamped message
            let pose = new geometry_msgs.PoseStamped();
            pose.header = odom.header;
            pose.pose = odom.pose.pose;
            this.encoder.pose_pub.publish(pose);

            // Update and publish the path
            this.encoder.path.header.stamp = rosTimestamp;
            this.encoder.path.poses.push(pose);
            this.encoder.path_pub.publish(this.encoder.path);

            // Update the last counts and time for the next iteration
            this.encoder.last_left_counts = this.encoder.left_counts;
            this.encoder.last_right_counts = this.encoder.right_counts;
            this.encoder.last_time = timestamp_sec;
        } catch (err) {
            rosnodejs.log.error(`Error processing message: ${err}`);
        }
    }

    convertOdometryToObj(data) {
        return {
            origin: {
                latitude: this.gps.initialLat,
                longitude: this.gps.initialLon
            },
            header: {
                seq: data.header.seq,
                stamp: {
                    secs: data.header.stamp.secs,
                    nsecs: data.header.stamp.nsecs
                },
                frame_id: data.header.frame_id
            },
            child_frame_id: data.child_frame_id,
            pose: {
                position: {
                    x: data.pose.pose.position.x,
                    y: data.pose.pose.position.y,
                    z: data.pose.pose.position.z
                },
                orientation: {
                    x: data.pose.pose.orientation.x,
                    y: data.pose.pose.orientation.y,
                    z: data.pose.pose.orientation.z,
                    w: data.pose.pose.orientation.w
                }
            },
            twist: {
                linear: {
                    x: data.twist.twist.linear.x,
                    y: data.twist.twist.linear.y,
                    z: data.twist.twist.linear.z
                },
                angular: {
                    x: data.twist.twist.angular.x,
                    y: data.twist.twist.angular.y,
                    z: data.twist.twist.angular.z
                }
            }
        };
    }

    subscribeOdometry(callback) {
        this.odometry_callbacks.push(callback);
    }
}
module.exports = PifRosMessagePublisher;