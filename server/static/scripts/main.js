console.log('Javascript works!!!');

let socket = io();

socket.on('connect', function(msg) {
    console.log(msg.data);
});

socket.on('traffic_data', function(msg) {
    console.log(msg.data + ': ' + msg.count);
})

window.addEventListener('unload', function() {
    socket.close();
});