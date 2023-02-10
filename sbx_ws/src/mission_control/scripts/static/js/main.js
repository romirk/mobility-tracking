const screen = document.getElementById('live');

// get colors from css
const Y_BLUE = getComputedStyle(document.documentElement).getPropertyValue('--y-blue');
const Y_SUCCESS = getComputedStyle(document.documentElement).getPropertyValue('--y-success');
const Y_WARNING = getComputedStyle(document.documentElement).getPropertyValue('--y-warning');
const Y_DANGER = getComputedStyle(document.documentElement).getPropertyValue('--y-danger');


function stamp_to_millis(stamp) {
    return stamp.secs * 1000 + stamp.nsecs / 1000000;
}


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
        this.boxes = []
        this.paint = false;
        this.last_box = 0;
        this.last_frame = 0;
        this.seq = 0;
        this.latency = { network: 0, processing: 0, total: 0 };

        this.canvas.width = 640;
        this.canvas.height = 480;
        this.ctx = this.canvas.getContext('2d');

        this.img = document.getElementById('live');

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

        this.los_screen();
        this.connect();
    }


    /**
     * Connect to ROSBridge
     */
    connect() {
        this.startTime = Date.now();
        this.relay.connect().then(() => this.setup(), () => this.reconnect("Failed to connect"));
    }

    /**
     * Handle disconnection.
    */
    reconnect(err) {
        this.los_screen();
        this.toastError("Mission Control", `Disconnected: ${err}`);
        this.relay.reset();
        setTimeout(() => this.connect(), 10000);
    }

    setup() {
        this.relay.createListener("/sbx/aqdata", "sensorbox/AQI", this.on_sensor.bind(this));
        this.relay.createListener("/sbx/result", "mission_control/Counts", this.on_count.bind(this));
        this.relay.createListener("/sbx/bboxed", "sensorbox/AnnotatedImage", this.on_image.bind(this));
        this.relay.createListener("/sbx/detect", "vision_msgs/Detection2D", this.on_detect.bind(this));
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
        // TODO calculate rate
        // this.log();
        this.update();
    }

    on_image(msg) {
        if (msg.img.header.seq <= this.seq) return;
        this.img.src = "data:image/jpeg;base64," + msg.img.data;
        this.on_box(msg);
        this.last_frame = Date.now();
        this.seq = msg.img.header.seq;
        this.latency.network = this.last_frame - stamp_to_millis(msg.img.header.stamp);
        this.latency.processing = stamp_to_millis(msg.header.stamp) - stamp_to_millis(msg.img.header.stamp);
        this.latency.total = this.latency.network + this.latency.processing;
        // console.log(this.latency);
        // this.paint = true;
    }

    on_box(msg) {
        if (msg.boxes.length === 0) return;
        const stamp = Date.now();
        for (const box of msg.boxes) {
            box.mstamp = stamp;
            box.color = Y_BLUE;
            this.boxes.push(box);
        }
        this.last_box = stamp;
        this.paint = true;
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
    }
    los_screen_off() {
        document.getElementById("los").style.display = "none";
    }

    box_anim() {
        if (!this.relay.connected) return this.reconnect("Connection lost!");

        const now = Date.now();
        if (now - this.last_frame > 1500 && !this.paint)
            this.los_screen();
        else if (this.paint || now - this.last_box > 100) {
            this.paint = false;
            this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
            this.los_screen_off();

            if (this.boxes.length > 0) {

                const new_boxes = [];

                for (const box of this.boxes) {
                    if (now - box.mstamp > 50) {
                        continue;
                    }
                    new_boxes.push(box);
                    this.draw_box(box.center.x, box.center.y, box.size_x, box.size_y);
                }
                this.boxes = new_boxes;
            }
        }

        window.requestAnimationFrame(this.box_anim.bind(this));
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
            dataset.data = [this.counts.cars, this.counts.trucks, this.counts.buses, this.counts.total - this.counts.cars - this.counts.trucks - this.counts.buses];
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
                dataset.data.push(this.counts.total - this.last_count);
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