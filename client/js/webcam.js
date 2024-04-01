
const connect_button = document.getElementById('connect-button');
const disconnect_button = document.getElementById('disconnect-button');
const host_input = document.getElementById('host-input')

const ros = new ROSLIB.Ros();

ros.on('error', function(error) {
    console.log(error);

    connect_button.disabled = false;
    host_input.disabled = false;
    disconnect_button.disabled = true;
});

ros.on('connection', function() {
    console.log('Connected to vehicle.');

    connect_button.disabled = true;
    host_input.disabled = true;
    disconnect_button.disabled = false;
});

ros.on('close', function() {
    console.log('Connection closed.');

    connect_button.disabled = false;
    host_input.disabled = false;
    disconnect_button.disabled = true;
});

function connectClicked() {
    connect_button.disabled = true;
    host_input.disabled = true;

    ros.connect('ws://' + host_input.value);
}

function disconnectClicked() {
    disconnect_button.disabled = true;

    ros.close();
}

const webcam_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/auto_vehicle/webcam/compressed',
    messageType: 'sensor_msgs/CompressedImage'
});

webcam_listener.subscribe(function(message) {
    document.getElementById('webcam-viewer').src = 'data:image/jpeg;base64,' + message.data;
});

const vision_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/auto_vehicle/vision/frame',
    messageType: 'sensor_msgs/CompressedImage'
});

vision_listener.subscribe(function(message) {
    document.getElementById('vision-viewer').src = 'data:image/jpeg;base64,' + message.data;
});