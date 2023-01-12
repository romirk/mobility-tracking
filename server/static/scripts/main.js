class SensorLive {
    constructor() {
        this.counter = 0;
        this.counts = {
            car: 0,
            bus: 0,
            truck: 0
        };
        this.sensor_data = {
            pm10: 0,
            pm25: 0,
            pm50: 0,
            pm100: 0,
            temperature: 0,
            humidity: 0,
            co2: 0
        }

        this.socket = io.connect();
        this.socket.on('connect', this.on_connect.bind(this));
        this.socket.on('disconnect', this.on_disconnect.bind(this));
        this.socket.on('sensor', this.on_sensor.bind(this));
        this.socket.on('detect', this.on_detect.bind(this));

        this.ctx = document.getElementById('myChart');
        this.chart = new Chart(this.ctx, {
            type: 'line', data: {
                labels: ['08:00', '09:00', '10:00', '11:00', '12:00', '13:00'], datasets: [{
                    label: 'Number of cars over time',
                    data: [65, 59, 80, 81, 56, 55, 40],
                    fill: false,
                    tension: 0.25,
                    borderWidth: 1
                }]
            }, options: {
                scales: {
                    y: {
                        beginAtZero: true
                    }
                }
            }
        });

        setInterval(this.update.bind(this), 1000);
    }

    on_connect() {
        console.log('Connected');
    }

    on_disconnect() {
        console.log('Disconnected');
    }

    on_sensor(data) {
        this.sensor_data = data;
    }

    on_detect(data) {
        this.counts.car = data.count;
        // this.counts = data.counts;
    }

    update() {
        // uncomment as you implement

        // document.getElementById('counter').innerHTML = this.counter;
        document.getElementById('counts_car').innerHTML = this.counts.car;
        document.getElementById('counts_bus').innerHTML = this.counts.bus;
        document.getElementById('counts_truck').innerHTML = this.counts.truck;
        // document.getElementById('sensor_pm10').innerHTML = this.sensor_data.pm10;
        // document.getElementById('sensor_pm25').innerHTML = this.sensor_data.pm25;
        // document.getElementById('sensor_pm50').innerHTML = this.sensor_data.pm50;
        // document.getElementById('sensor_pm100').innerHTML = this.sensor_data.pm100;
        // document.getElementById('sensor_temperature').innerHTML = this.sensor_data.temperature;
        // document.getElementById('sensor_humidity').innerHTML = this.sensor_data.humidity;
        // document.getElementById('sensor_co2').innerHTML = this.sensor_data.co2;

        // Update chart
        // TODO
    }

    log() {
        console.log(this.counter)
        console.log(this.counts);
        console.log(this.sensor_data);
    }
}

const sensor_live = new SensorLive();
sensor_live.log()