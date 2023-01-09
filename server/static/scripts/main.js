console.log('Javascript works!!!');

let socket = io();

socket.on('connect', function(msg) {
    console.log(msg.data);
});

socket.on('traffic_data', function(msg) {
    console.log(msg.data + ': ' + msg.count);
})

const ctx = document.getElementById('myChart');

let chart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: ['08:00', '09:00', '10:00', '11:00', '12:00', '13:00'],
        datasets: [{
            label: 'Number of cars over time',
            data: [65, 59, 80, 81, 56, 55, 40],
            fill: false,
            tension: 0.25,
            borderWidth: 1
        }]
    },
    options: {
        scales: {
            y: {
                beginAtZero: true
            }
        }
    }
});