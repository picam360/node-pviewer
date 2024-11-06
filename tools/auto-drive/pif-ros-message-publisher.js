const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const nav_msgs = rosnodejs.require('nav_msgs').msg;

class PifRosMessagePublisher {
    constructor() {
        this.isInitialized = false;
        this.isInitializing = false;

        // wheel vars
        this.bWheelUpdate = false;
        this.wheelLeftCount = 0;
        this.wheelRightCount = 0;
        this.wheelTimestampSec = null;

        this.bGpsUpdate = false;
        this.gpsX = 0;
        this.gpsY = 0;
        this.gpsRtksate = 0;
        this.gpsHdop = 0;
        this.gpsTimestampSec = null;
        this.gpsNmea = null;
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

        this.nh = rosnodejs.nh;
        this.std_msgs = rosnodejs.require('std_msgs').msg;

        // Create publishers
        this.wheelLeftPub = this.nh.advertise('/left_wheel_counts', std_msgs.Int32, {
            queueSize: 10
        });
        this.wheelRightPub = this.nh.advertise('/right_wheel_counts', std_msgs.Int32, {
            queueSize: 10
        });
        this.wheelPub = this.nh.advertise('/wheel_counts', std_msgs.Float64MultiArray, {
            queueSize: 10
        });
        this.gpsPub = this.nh.advertise('/gps_nmea', std_msgs.String, {
            queueSize: 10
        });

        this.publishMessages();
    }

    updateWheelCount(left, right, timestampSec) {
        this.wheelLeftCount = left;
        this.wheelRightCount = right;
        this.wheelTimestampSec = timestampSec;
        this.bWheelUpdate = true;
    }

    updateGpsPos(x, y, rtkstate, hdop, timestampSec) {
        this.gpsX = x;
        this.gpsY = y;
        this.gpsRtksate = rtkstate;
        this.gpsHdop = hdop;
        this.timestampSec = timestampSec;
        this.gpsTimestampSec = true;
    }

    updateGpsNmea(nmea, timestamp) {
        this.gpsNmea = nmea;
        this.gpsTimestampSec = timestamp;
        this.bGpsUpdate = true;
    }

    publishMessages() {
        if (!this.isInitialized) {
            return;
        }

        if (this.bGpsUpdate) {
            this.bGpsUpdate = false;
            this.gpsPub.publish({ data: this.gpsNmea + "@" + this.gpsTimestampSec.toString() });
        }

        if (this.bWheelUpdate) {
            this.bWheelUpdate = false;
            const msg = {
                layout: {
                    dim: [{
                        label: "data",
                        size: 3,
                        stride: 3
                    }],
                    data_offset: 0
                },
                data: [
                    Number(this.wheelTimestampSec),
                    Number(this.wheelLeftCount),
                    Number(this.wheelRightCount)
                ]
            };
            this.wheelPub.publish(msg);
        }
    }

    convertOdometryToObj(data) {
        return {
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
        this.topic1Sub = this.nh.subscribe('/odometry/filtered', nav_msgs.Odometry, (data) => {
            callback(this.convertOdometryToObj(data));
        });
    }
}
module.exports = PifRosMessagePublisher;