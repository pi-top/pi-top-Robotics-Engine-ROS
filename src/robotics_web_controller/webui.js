var twist;
var twist2;
var cmdVel;
var cmdVel2;
var publishImmidiately = true;
var robot_IP;
var manager;
var managerPanTi;  //for pan and tilt
var teleop;
var ros;

function moveAction(linear, angular) {
    if (linear !== undefined && angular !== undefined) {
        twist.linear.x = linear;
        twist.angular.z = angular;
    } else {
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
    // console.log(linear)
    cmdVel.publish(twist);
}

function moveAction_PanTi(angular, angular2) {
    if (angular !== undefined && angular2 !== undefined) {
        twist2.angular.x = angular;
        twist2.angular.z = angular2;
    } else {
        twist2.angular.x = 0;
        twist2.angular.z = 0;
    }
    // console.log(linear)
    cmdVel2.publish(twist2);
}

function initVelocityPublisher() {
    // Init message with zero values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    // Init topic object
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
    // Register publisher within ROS system
    cmdVel.advertise();
}

function initVelocityPublisher_PanTi() {
    // Init message with zero values.
    twist2 = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    // Init topic object
    cmdVel2 = new ROSLIB.Topic({
        ros: ros,
        name: '/pan_tilt/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
    // Register publisher within ROS system
    cmdVel2.advertise();
}

function initTeleopKeyboard() {
    // Use w, s, a, d keys to drive your robot

    // Check if keyboard controller was aready created
    if (teleop == null) {
        // Initialize the teleop.
        teleop = new KEYBOARDTELEOP.Teleop({
            ros: ros,
            topic: '/cmd_vel'
        });
    }

    // Add event listener for slider moves
    robotSpeedRange = document.getElementById("robot-speed");
    robotSpeedRange.oninput = function () {
        teleop.scale = robotSpeedRange.value / 100
    }
}

function createJoystick() {
    // Check if joystick was aready created
    if (manager == null) {
        joystickContainer = document.getElementById('joystick');
        // joystick configuration, if you want to adjust joystick, refer to:
        // https://yoannmoinet.github.io/nipplejs/
        var options = {
            zone: joystickContainer,
            position: { left: 30 + '%', top: 105 + 'px' },
            mode: 'static',
            size: 200,
            color: '#0066ff',
            restJoystick: true
        };
        manager = nipplejs.create(options);
        // event listener for joystick move
        manager.on('move', function (evt, nipple) {
            // nipplejs returns direction is screen coordinates
            // we need to rotate it, that dragging towards screen top will move robot forward
            var direction = nipple.angle.degree - 90;
            if (direction > 180) {
                direction = -(450 - nipple.angle.degree);
            }
            // convert angles to radians and scale linear and angular speed
            // adjust if you want robot to drive faster or slower
            var lin = Math.cos(direction / 57.29) * nipple.distance * 0.008;
            var ang = Math.sin(direction / 57.29) * nipple.distance * 0.03;
            // nipplejs is triggering events when joystick moves each pixel
            // we need delay between consecutive message publications to
            // prevent system from being flooded by messages
            // events triggered earlier than 50ms after last publication will be dropped 
            if (publishImmidiately) {
                publishImmidiately = false;
                moveAction(lin, ang);
                setTimeout(function () {
                    publishImmidiately = true;
                }, 50);
            }
        });
        // event litener for joystick release, always send stop message
        manager.on('end', function () {
            // send twice to make sure it gets through
            moveAction(0, 0);
            moveAction(0, 0);
        });
    }
}

function createJoystick_PanTi() {
    // Check if joystick was aready created
    if (managerPanTi == null) {
        joystickContainer = document.getElementById('joystick1');
        // joystick configuration, if you want to adjust joystick, refer to:
        // https://yoannmoinet.github.io/nipplejs/
        var options = {
            zone: joystickContainer,
            position: { left: 70 + '%', top: 105 + 'px' },
            mode: 'static',
            size: 200,
            color: '#0066ff',
            restJoystick: true
        };
        managerPanTi = nipplejs.create(options);
        // event listener for joystick move
        managerPanTi.on('move', function (evt, nipple) {
            // nipplejs returns direction is screen coordinates
            // we need to rotate it, that dragging towards screen top will move robot forward
            var direction = nipple.angle.degree - 90;
            if (direction > 180) {
                direction = -(450 - nipple.angle.degree);
            }
            // convert angles to radians and scale linear and angular speed
            // adjust if you want robot to drive faster or slower
            var ang = Math.cos(direction / 57.29) * nipple.distance * 0.03;
            var ang2 = Math.sin(direction / 57.29) * nipple.distance * 0.03;
            // nipplejs is triggering events when joystick moves each pixel
            // we need delay between consecutive message publications to
            // prevent system from being flooded by messages
            // events triggered earlier than 50ms after last publication will be dropped 
            if (publishImmidiately) {
                publishImmidiately = false;
                moveAction_PanTi(ang, ang2);
                setTimeout(function () {
                    publishImmidiately = true;
                }, 50);
            }
        });
        // event litener for joystick release, always send stop message
        managerPanTi.on('end', function () {
            // send twice to make sure it gets through
            moveAction_PanTi(0, 0);
            moveAction_PanTi(0, 0);

        });
    }
}


window.onload = function () {
    // determine robot address automatically
    robot_IP = location.hostname;
    // set robot address statically
    // robot_IP = "192.168.1.183";

    // // Init handle for rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9090"
    });

    initVelocityPublisher();
	initVelocityPublisher_PanTi();
	initTeleopKeyboard();
	createJoystick();
    createJoystick_PanTi();

    // get handle for video placeholder
    video = document.getElementById('video');
    // Populate video source 
    video.src = "http://" + robot_IP + ":8080/stream?topic=/cv_camera/image_raw&type=ros_compressed"; //&quality=20

    video.onload = function () {
        // use if joystick and keyboard controls should only be available when video is correctly loaded
        createJoystick();
        createJoystick_PanTi();
    };
}
