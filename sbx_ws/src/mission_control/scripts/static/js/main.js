const screen = document.getElementById('live');

// get colors from css
const Y_BLUE = getComputedStyle(document.documentElement).getPropertyValue('--y-blue');
const Y_DBLUE = getComputedStyle(document.documentElement).getPropertyValue('--y-dblue');
const Y_SUCCESS = getComputedStyle(document.documentElement).getPropertyValue('--y-success');
const Y_WARNING = getComputedStyle(document.documentElement).getPropertyValue('--y-warning');
const Y_DANGER = getComputedStyle(document.documentElement).getPropertyValue('--y-danger');


function stamp_to_millis(stamp) {
    return stamp.secs * 1000 + stamp.nsecs / 1000000;
}
function prepend(value, array) {
    var newArray = array.slice();
    newArray.unshift(value);
    return newArray;
}

Chart.defaults.backgroundColor = Y_DBLUE;
// Chart.defaults.borderColor = Y_BLUE;
Chart.defaults.color = 'white';

class SensorLive {

    canvas = document.getElementById('canvas');
    toastElement = new bootstrap.Toast(document.getElementById("errorToast"));

    constructor() {
        this.counts = {
            total: 0, cars: 0, trucks: 0, buses: 0, motorcycles: 0, bicycles: 0,
        };
        this.sensor_data = {
            pm10: 0, pm25: 0, pm50: 0, pm100: 0, tmp: 0, hum: 0, co2: 0
        };
        this.last_count = -1;
        this.timeline = [];
        this.boxes = []
        this.paint = true;
        this.last_box = 0;
        this.last_frame = 0;
        this.seq = 0;
        this.los = false;
        this.latency = { network: 0, processing: 0, total: 0 };

        this.canvas.width = 640;
        this.canvas.height = 480;
        this.ctx = this.canvas.getContext('2d');

        this.img = document.getElementById('live');

        this.relay = new Relay(window.location.hostname, 9090);

        this.initGraphs();

        this.los_screen();
        this.connect();
    }

    initGraphs() {
        const lineChartConfig = {
            type: 'line',
            data: {
                datasets: [{
                    label: 'measurement over time',
                    data: [],
                    fill: false,
                    tension: 0.35,
                    borderWidth: 3
                }]
            },
            options: {
                scales: {
                    x: {
                        type: 'time',
                    },
                    y: {
                        beginAtZero: true,
                    }
                },
                responsive: true,
                maintainAspectRatio: false
            }
        };

        this.ctxvehiclesOverTime = document.getElementById('numberOfVehicles');
        this.vehiclesOverTime = new Chart(this.ctxvehiclesOverTime, structuredClone(lineChartConfig));
        this.vehiclesOverTime.data.datasets[0].label = 'Number of vehicles over time';
        // this.vehiclesOverTime.data.datasets[0].borderColor = Y_BLUE;

        this.ctxNumberOfVehicles = document.getElementById('typesOfVehicles');
        this.numberOfVehicles = new Chart(this.ctxNumberOfVehicles, {
            type: 'doughnut',
            data: {
                labels: ['Cars', 'Trucks', 'Buses', 'Other'],
                datasets: [{
                    label: 'Types of vehicles',
                    data: [0.0000001, 0.0000001, 0.0000001, 0.0000001],
                    backgroundColor: ['rgb(2, 159, 227)', 'rgb(26,122,165)', 'rgb(31,84,108)'],
                    hoverOffset: 4
                }]
            }, options: {
                responsive: true, maintainAspectRatio: false
            }
        });

        this.ctxPM10 = document.getElementById('pm10chart');
        this.pm10Chart = new Chart(this.ctxPM10, structuredClone(lineChartConfig));
        this.pm10Chart.data.datasets[0].label = 'PM1.0 over time';
        // this.pm10Chart.data.datasets[0].borderColor = 'rgb(2, 159, 227)';

        this.ctxPM25 = document.getElementById('pm25chart');
        this.pm25Chart = new Chart(this.ctxPM25, structuredClone(lineChartConfig));
        this.pm25Chart.data.datasets[0].label = 'PM2.5 over time';
        // this.pm25Chart.data.datasets[0].borderColor = 'rgb(26,122,165)';

        // this.ctxPM50 = document.getElementById('pm50chart');
        // this.pm50Chart = new Chart(this.ctxPM50, lineChartconfig);
        // this.pm50Chart.data.datasets[0].label = 'PM5.0 over time';

        // this.ctxPM100 = document.getElementById('pm100chart');
        // this.pm100Chart = new Chart(this.ctxPM100, lineChartconfig);
        // this.pm100Chart.data.datasets[0].label = 'PM10.0 over time';
        // this.pm100Chart.data.datasets[0].borderColor = 'rgb(31,84,108)';

        this.ctxTemperature = document.getElementById('temperaturechart');
        this.temperatureChart = new Chart(this.ctxTemperature, structuredClone(lineChartConfig));
        this.temperatureChart.data.datasets[0].label = 'Temperature over time';
        // this.temperatureChart.data.datasets[0].borderColor = 'rgb(2, 159, 227)';

        this.ctxHumidity = document.getElementById('humiditychart');
        this.humidityChart = new Chart(this.ctxHumidity, structuredClone(lineChartConfig));
        this.humidityChart.data.datasets[0].label = 'Humidity over time';
        // this.humidityChart.data.datasets[0].borderColor = 'rgb(26,122,165)';

        this.ctxCO2 = document.getElementById('co2chart');
        this.co2Chart = new Chart(this.ctxCO2, structuredClone(lineChartConfig));
        this.co2Chart.data.datasets[0].label = 'CO2 over time';
        // this.co2Chart.data.datasets[0].borderColor = 'rgb(31,84,108)';

        // TODO this.ctxLatency = document.getElementById('latencychart');
    }


    /**
     * Connect to ROSBridge
     */
    connect() {
        this.startTime = Date.now();
        this.los_screen_off();
        this.toastInfo("Mission Control", "Connecting...");
        this.relay.connect().then(() => this.setup(), () => this.reconnect("Failed to connect"));
    }

    /**
     * Handle disconnection.ticks: {
                        //     callback: function (value, index, ticks) {
                        //         console.log(value);
                        //         const time = new Date(value);
                        //         return `${time.getHours()}:${time.getMinutes()}:${time.getSeconds()}`;
                        //     }
                        // }
    */
    reconnect(err) {
        this.los_screen();
        this.toastError("Mission Control", `Disconnected: ${err}`);
        this.relay.reset();
        setTimeout(() => this.connect(), 10000);
    }

    setup() {
        this.toastSuccess("Mission Control", "Connected");
        this.relay.createListener("/sbx/aqdata", "sensorbox/AQI", this.on_sensor.bind(this));
        this.relay.createListener("/sbx/result", "mission_control/Counts", this.on_count.bind(this));
        this.relay.createListener("/sbx/bboxed", "sensorbox/AnnotatedImage", this.on_image.bind(this));
        this.relay.createListener("/sbx/detect", "vision_msgs/Detection2D", this.on_detect.bind(this));

        this.relay.createServiceClient("/sbx/timeline", "mission_control/Timeline");
        this.load_timeline();


        setInterval(this.updateGraphs.bind(this), 10000);
        window.requestAnimationFrame(this.box_anim.bind(this));
    }

    on_sensor(msg) {
        this.sensor_data = msg;
        // this.log();
        this.update();
    }

    on_detect(msg) {
        // this.draw_box(msg.bbox.center.x, msg.bbox.center.y, msg.bbox.size_x, msg.bbox.size_y, (48, 209, 88));
        const box = msg.bbox;
        box.mstamp = Date.now();
        box.color = Y_SUCCESS;
        this.boxes.push(box);
        console.log("box", box);
    }

    on_count(msg) {
        this.counts = msg;
        if (this.last_count === -1) {
            this.last_count = msg.total;
        }
        this.timeline.push({ stamp: { secs: Date.now() / 1000, nsecs: 0 }, counts: msg });
        this.update();
    }

    on_image(msg) {
        if (msg.img.header.seq <= this.seq) return;
        this.img.src = "data:image/jpeg;base64," + msg.img.data;
        this.on_box(msg);
        this.last_frame = Date.now();
        this.paint = true;

        this.seq = msg.img.header.seq;
        // this.latency.network = this.last_frame - stamp_to_millis(msg.img.header.stamp);
        // this.latency.processing = stamp_to_millis(msg.header.stamp) - stamp_to_millis(msg.img.header.stamp);
        // this.latency.total = this.latency.network + this.latency.processing;
    }

    on_box(msg) {
        if (msg.boxes.length === 0) return;
        const stamp = Date.now();
        for (const box of msg.boxes) {
            box.mstamp = stamp;
            box.color = "#1ccbed";
            this.boxes.push(box);
        }
        this.last_box = stamp;
    }

    load_timeline() {
        this.relay.callService("/sbx/timeline", {}).then((res) => {
            if (res.timeline.length === 0) return;
            const timeline = res.timeline.slice(0, 50).reverse();
            this.timeline = timeline;
            const last = timeline[timeline.length - 1];
            const first = timeline[0];
            const start = stamp_to_millis(first.stamp);
            const end = stamp_to_millis(last.stamp);
            const diff = end - start;
            const step = diff / 100;
            let prev = timeline[0].counts.total;

            this.vehiclesOverTime.data.datasets[0].data = [];
            let p = 0;
            for (let i = 0; i < 100; i++) {
                const time = start + step * i;
                let count = 0;

                while (p < timeline.length && stamp_to_millis(timeline[p].stamp) < time) {
                    count += timeline[p].counts.total - prev;
                    prev = timeline[p].counts.total;
                    p++;
                }
                this.vehiclesOverTime.data.datasets[0].data.push({ x: new Date(time), y: count });
            }

            console.log(`loaded ${timeline.length} counts`);
            this.vehiclesOverTime.update();
        }).catch((err) => {
            this.toastError("Mission Control", `Failed to load timeline: ${err}`)
        });
    }

    computeRate() {
        let rate = 0;
        const diff = stamp_to_millis(this.timeline[this.timeline.length - 1].stamp) - stamp_to_millis(this.timeline[this.timeline.length - 10].stamp);
        this.timeline.slice(-10).forEach((item) => {
            rate += item.counts.total;
        });
        return rate / diff;
    }


    draw_box(center_x, center_y, size_x, size_y, color = Y_BLUE) {
        // this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        const x = center_x - size_x / 2;
        const y = center_y - size_y / 2;
        // console.log(x, y, size_x, size_y);
        this.ctx.lineWidth = 4;
        this.ctx.strokeStyle = color;
        this.ctx.strokeRect(center_x - size_x / 2, center_y - size_y / 2, size_x, size_y);
        // this.ctx.fillRect(center_x - size_x / 2, center_y - size_y / 2, size_x, size_y);
        // this.ctx.stroke();
    }

    los_screen() {
        document.getElementById("los").style.display = "grid";
        this.los = true;
        // console.log("los");
    }
    los_screen_off() {
        document.getElementById("los").style.display = "none";
        this.los = false;
        // console.log("los off");
    }

    box_anim() {
        if (!this.relay.connected) return this.reconnect("Connection lost!");

        const now = Date.now();
        if (!this.los) {
            if (now - this.last_frame > 5000 && !this.paint) {
                this.boxes = [];
                this.los_screen();
                this.toastWarning("Mission Control", "No frames received");
            }
            else if (this.boxes.length > 0 || now - this.last_box > 100) {
                this.paint = false;
                this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

                if (this.boxes.length > 0) {

                    const new_boxes = [];

                    for (const box of this.boxes) {
                        if (now - box.mstamp > 50) {
                            continue;
                        }
                        new_boxes.push(box);
                        this.draw_box(box.center.x, box.center.y, box.size_x, box.size_y, box.color);
                    }
                    this.boxes = new_boxes;
                }
            }
        }
        else if (this.paint) {
            this.los_screen_off();
        }

        window.requestAnimationFrame(this.box_anim.bind(this));
    }


    update() {
        // uncomment as you implement

        document.getElementById('total_count').innerHTML = this.computeRate().toFixed(2);
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
            dataset.data = [this.counts.cars, this.counts.trucks, this.counts.buses, this.counts.total - this.counts.cars - this.counts.trucks - this.counts.buses];
        });

        this.numberOfVehicles.update();
    }

    updateGraphs() {
        // Update chart [VEHICLES OVER TIME]
        let date = new Date()

        this.vehiclesOverTime.data.datasets.forEach((dataset) => {
            dataset.data.shift();
            if (this.last_count === -1) {
                dataset.data.push({ x: date, y: 0 });
            } else {
                dataset.data.push({ x: date, y: this.counts.total - this.last_count });
                this.last_count = this.counts.total;
            }
        });

        this.vehiclesOverTime.update();
    }

    // utilities

    /**
     * Displays a temporary notification at the bottom of the screen.
     * @param title
     * @param module
     * @param msg
     * @param color
     */
    toast(title, module, msg, color) {
        $("#toast-small").text(module);
        $("#toast-msg").text(msg);
        $("#toast-title").text(title);
        $("#toast-rect").attr("fill", color);
        this.toastElement.show();
    }

    toastError(module, message) {
        this.toast("Error", module, message, "#dc3545");
    }

    toastInfo(module, message) {
        this.toast("Info", module, message, "#0d6efd");
    }

    toastSuccess(module, message) {
        this.toast("Success", module, message, Y_SUCCESS);
    }

    toastWarning(module, message) {
        this.toast("Warning", module, message, Y_WARNING);
    }
}


const sensor_live = new SensorLive();