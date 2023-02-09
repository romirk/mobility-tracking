const screen = document.getElementById('live');
class SensorLive {
    constructor() {
        this.counter = 0;
        this.counts = {
            total: 0, cars: 0, trucks: 0, buses: 0, motorcycles: 0, bicycles: 0,
        };
        this.sensor_data = {
            pm10: 0, pm25: 0, pm50: 0, pm100: 0, tmp: 0, hum: 0, co2: 0
        };
        this.last_count = -1;

        this.canvas = document.getElementById('canvas');
        this.ctx = this.canvas.getContext('2d');
        this.canvas.width = 640;
        this.canvas.height = 480;

        this.relay = new Relay(window.location.hostname, 9090);


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

        // setInterval(this.update.bind(this), 200);
        setInterval(this.updateGraphs.bind(this), 10000);

        this.relay.connect().then(this.relay_setup.bind(this));
    }

    relay_setup() {
        this.relay.createListener("/sbx/aqdata", "sensorbox/AQI", this.on_sensor.bind(this));
        this.relay.createListener("/sbx/result", "mission_control/Counts", this.on_detect.bind(this));
        this.relay.createListener("/sbx/camera/color/image_raw/compressed", "sensor_msgs/CompressedImage", this.on_image.bind(this));
        this.relay.createListener("/sbx/bounds", "vision_msgs/BoundingBox2DArray", this.on_box.bind(this));
    }

    on_sensor(data) {
        this.sensor_data = data;
        // this.log();
        this.update();
    }

    on_detect(data) {
        this.counts = data;

        // TODO calculate rate
        // this.log();
        this.update();
    }

    on_image(data) {
        screen.src = "data:image/jpeg;base64," + data.data;
    }

    on_box(data) {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        this.ctx.lineWidth = "4";
        this.ctx.strokeStyle = "#1cbbed";
        for (const box of data.boxes) {
            this.ctx.rect(box.center.x - box.size_x / 2, box.center.y - box.size_y / 2, box.size_x, box.size_y);
        }
        this.ctx.stroke();
        console.log(data);
    }


    update() {
        // uncomment as you implement

        document.getElementById('total_count').innerHTML = this.counts.total;
        // document.getElementById('counts_car').innerHTML = this.counts.cars;
        // document.getElementById('counts_bus').innerHTML = this.counts.buses;
        // document.getElementById('counts_truck').innerHTML = this.counts.trucks;
        document.getElementById('total_count_2').innerHTML = this.counts.total;
        document.getElementById('counts_car_2').innerHTML = this.counts.cars;
        document.getElementById('counts_bus_2').innerHTML = this.counts.buses;
        document.getElementById('counts_truck_2').innerHTML = this.counts.trucks;

        // sensor data

        document.getElementById('pm10').innerHTML = Math.round(this.sensor_data.pm10 * 100) / 100;
        document.getElementById('pm25').innerHTML = Math.round(this.sensor_data.pm25 * 100) / 100;
        // document.getElementById('pm50').innerHTML = this.sensor_data.pm50;
        // document.getElementById('pm100').innerHTML = this.sensor_data.pm100;
        document.getElementById('tmp').innerHTML = Math.round(this.sensor_data.tmp * 100) / 100;
        document.getElementById('hum').innerHTML = Math.round(this.sensor_data.hum * 100) / 100;
        document.getElementById('co2').innerHTML = Math.round(this.sensor_data.co2 * 100) / 100;

        this.numberOfVehicles.data.datasets.forEach((dataset) => {
            dataset.data = [this.counts.cars, this.counts.trucks, this.counts.buses];
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
        console.log(this.counts);
        console.log(this.sensor_data);
    }
}


const sensor_live = new SensorLive();
sensor_live.log()

window.onkeyup = function (e) {
    const key = e.code;

    if (key === 'KeyL') {
        let old_data = sensor_live.counts;
        console.log(old_data);
        old_data.car.total += 1;
        sensor_live.on_detect({ counts: old_data });
    }
}