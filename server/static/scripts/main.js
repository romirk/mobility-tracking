class SensorLive {
    constructor() {
        this.counter = 0;
        this.counts = {
            car: {total: 0}, bus: {total: 0}, truck: {total: 0}
        };
        this.sensor_data = {
            pm10: 0, pm25: 0, pm50: 0, pm100: 0, temperature: 0, humidity: 0, co2: 0
        };
        this.last_count = -1;

        this.socket = io.connect();
        this.socket.on('connect', this.on_connect.bind(this));
        this.socket.on('disconnect', this.on_disconnect.bind(this));
        this.socket.on('sensor', this.on_sensor.bind(this));
        this.socket.on('detect', this.on_detect.bind(this));

        this.ctxvehiclesOverTime = document.getElementById('numberOfVehicles');
        this.vehiclesOverTime = new Chart(this.ctxvehiclesOverTime, {
            type: 'line',
            data: {
                labels: ["-", "-", "-", "-", "-", "-", "-", "-", "-", "-"],
                datasets: [{
                    label: 'Number of vehicles over time (updates every 10 seconds)',
                    data: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    fill: false,
                    tension: 0.35,
                    borderWidth: 3
                }]
            }, options: {
                scales: {
                    y: {
                        beginAtZero: true
                    }
                }, responsive: true, maintainAspectRatio: false
            }
        });

        this.ctxNumberOfVehicles = document.getElementById('typesOfVehicles');
        this.numberOfVehicles = new Chart(this.ctxNumberOfVehicles, {
            type: 'doughnut',
            data: {
                labels: ['Cars', 'Trucks', 'Buses'],
                datasets: [{
                    label: 'Types of vehicles',
                    data: [0.0000001, 0.0000001, 0.0000001],
                    backgroundColor: ['rgb(2, 159, 227)', 'rgb(26,122,165)', 'rgb(31,84,108)'],
                    hoverOffset: 4
                }]
            }, options: {
                responsive: true, maintainAspectRatio: false
            }
        });

        setInterval(this.update.bind(this), 1000);
        setInterval(this.updateGraphs.bind(this), 10000);
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
        console.log(data)
        this.counts = data.counts;
        this.counter = this.counts.car.total + this.counts.bus.total + this.counts.truck.total;
    }

    update() {
        // uncomment as you implement

        // document.getElementById('counter').innerHTML = this.counter;
        document.getElementById('total_count').innerHTML = this.counter;
        document.getElementById('counts_car').innerHTML = this.counts.car.total;
        document.getElementById('counts_bus').innerHTML = this.counts.bus.total;
        document.getElementById('counts_truck').innerHTML = this.counts.truck.total;
        document.getElementById('total_count_2').innerHTML = this.counter;
        document.getElementById('counts_car_2').innerHTML = this.counts.car.total;
        document.getElementById('counts_bus_2').innerHTML = this.counts.bus.total;
        document.getElementById('counts_truck_2').innerHTML = this.counts.truck.total;
        // document.getElementById('sensor_pm10').innerHTML = this.sensor_data.pm10;
        // document.getElementById('sensor_pm25').innerHTML = this.sensor_data.pm25;
        // document.getElementById('sensor_pm50').innerHTML = this.sensor_data.pm50;
        // document.getElementById('sensor_pm100').innerHTML = this.sensor_data.pm100;
        // document.getElementById('sensor_temperature').innerHTML = this.sensor_data.temperature;
        // document.getElementById('sensor_humidity').innerHTML = this.sensor_data.humidity;
        // document.getElementById('sensor_co2').innerHTML = this.sensor_data.co2;

        this.numberOfVehicles.data.datasets.forEach((dataset) => {
            dataset.data = [this.counts.car.total, this.counts.truck.total, this.counts.bus.total];
        });

        this.numberOfVehicles.update();
    }

    updateGraphs() {
        // Update chart [VEHICLES OVER TIME]
        let date = new Date()

        this.vehiclesOverTime.data.labels.shift();
        this.vehiclesOverTime.data.labels.push(
            date.getHours().toString() + ":" +
            date.getMinutes().toString() + ":" +
            date.getSeconds().toString()
        );

        this.vehiclesOverTime.data.datasets.forEach((dataset) => {
            dataset.data.shift();
            if (this.last_count === -1) {
                dataset.data.push(0);
            } else {
                dataset.data.push(this.counter - this.last_count);
            }
            this.last_count = this.counter;
        });

        this.vehiclesOverTime.update();
    }

    log() {
        console.log(this.counter)
        console.log(this.counts);
        console.log(this.sensor_data);
    }
}

const sensor_live = new SensorLive();
sensor_live.log()