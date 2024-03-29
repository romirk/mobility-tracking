const screen = document.getElementById('live');

// get colors from css
const Y_BLUE = getComputedStyle(document.documentElement).getPropertyValue('--y-blue');
const Y_DBLUE = getComputedStyle(document.documentElement).getPropertyValue('--y-dblue');
const Y_SUCCESS = getComputedStyle(document.documentElement).getPropertyValue('--y-success');
const Y_WARNING = getComputedStyle(document.documentElement).getPropertyValue('--y-warning');
const Y_DANGER = getComputedStyle(document.documentElement).getPropertyValue('--y-danger');

const triggerTabList = document.querySelectorAll('#selector button')
const tabs = [];

triggerTabList.forEach(triggerEl => {
    const tabTrigger = new bootstrap.Tab(triggerEl)

    tabs.push(tabTrigger);
});


function stampToMillis(stamp) {
    return stamp.secs * 1000 + stamp.nsecs / 1000000;
}

function prepend(value, array) {
    var newArray = array.slice();
    newArray.unshift(value);
    return newArray;
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

Chart.defaults.backgroundColor = Y_DBLUE;
// Chart.defaults.borderColor = Y_BLUE;
Chart.defaults.color = 'white';

class SensorLive {

    canvas = document.getElementById('canvas');
    toastElement = new bootstrap.Toast(document.getElementById("errorToast"));

    constructor() {
        this.connected = false;

        this.counts = {
            total: 0, cars: 0, trucks: 0, buses: 0, motorcycles: 0, bicycles: 0,
        };
        this.timeline = [];
        this.lastCount = -1;
        this.rate = 0;
        this.boxes = []


        this.sensorData = {
            pm10: 0, pm25: 0, pm50: 0, pm100: 0, tmp: 0, hum: 0, co2: 0
        };
        this.prevSensorData = {
            pm10: 0, pm25: 0, pm50: 0, pm100: 0, tmp: 0, hum: 0, co2: 0
        };
        this.sensorTimeline = {
            pm10: [], pm25: [], pm50: [], pm100: [], tmp: [], hum: [], co2: []
        };


        this.paint = true;
        this.lastBox = 0;
        this.lastFrame = 0;
        this.seq = 0;
        this.los = false;
        this.frozen = false;
        this.latency = {network: 0, processing: 0, total: 0};

        this.canvas.width = 640;
        this.canvas.height = 480;
        this.ctx = this.canvas.getContext('2d');

        this.img = document.getElementById('live');

        this.relay = new Relay(window.location.hostname, 9090);

        this.initGraphs();
        this.graphViews = [0.1, 1, 3, 8];
        this.graphView = 0;

        this.losScreen();
        this.connect();
    }

    initGraphs() {
        const lineChartConfig = {
            type: 'line', data: {
                datasets: [{
                    label: 'measurement over time', data: [], fill: false, tension: 0.35, borderWidth: 3
                }]
            }, options: {
                scales: {
                    x: {
                        type: 'time',
                    }, y: {
                        beginAtZero: true,
                    }
                }, responsive: true, maintainAspectRatio: false
            }
        };

        this.ctxvehiclesOverTime = document.getElementById('numberOfVehicles');
        this.vehiclesOverTime = new Chart(this.ctxvehiclesOverTime, structuredClone(lineChartConfig));
        // this.vehiclesOverTime.data.datasets[0].borderColor = Y_BLUE;

        this.ctxNumberOfVehicles = document.getElementById('typesOfVehicles');
        this.numberOfVehicles = new Chart(this.ctxNumberOfVehicles, {
            type: 'doughnut', data: {
                labels: ['Cars', 'Trucks', 'Buses', 'Other'], datasets: [{
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
        this.toastInfo("Mission Control", "Connecting...");
        this.relay.connect()
            .then(this.rosSetup.bind(this))
            .then(() => sleep(2000)) // wait for connection to stabilize
            .then(this.setup.bind(this))
            .catch(() => this.reconnect("Failed to connect"));
    }

    /**
     * Handle disconnection.
     */
    reconnect(err) {
        this.connected = false;
        tabs[0].show();
        this.losScreen();
        this.navOut();
        this.toastError("Mission Control", `Disconnected: ${err}`);
        this.relay.reset();
        setTimeout(() => this.connect(), 10000);
    }

    rosSetup() {
        this.relay.createListener("/sbx/aqdata", "sensorbox/AQI", this.onSensor.bind(this));
        this.relay.createListener("/sbx/result", "mission_control/Counts", this.onCount.bind(this));
        this.relay.createListener("/sbx/bboxed", "sensorbox/AnnotatedImage", this.onImage.bind(this));
        this.relay.createListener("/sbx/detect", "vision_msgs/Detection2D", this.onDetect.bind(this));
        this.relay.createServiceClient("/sbx/timetravel/history", "mission_control/Timeline");
    }

    setup() {
        this.connected = true;
        this.toastSuccess("Mission Control", "Connected");
        // this.losScreenOff();
        this.navIn();
        this.loadTimeline();

        this.prevSensorData = structuredClone(this.sensorData);

        setInterval(() => {
            this.graphView = (this.graphView + 1) % this.graphViews.length;
            this.updateGraphs()
        }, 10000);
        window.requestAnimationFrame(this.getFrame.bind(this));
    }

    onSensor(msg) {
        this.sensorData = msg;
        const time = (new Date()).getTime();
        this.sensorTimeline.tmp.push({x: time, y: this.sensorData.tmp});
        this.sensorTimeline.hum.push({x: time, y: this.sensorData.hum});
        this.sensorTimeline.co2.push({x: time, y: this.sensorData.co2});
        this.sensorTimeline.pm10.push({x: time, y: this.sensorData.pm10});
        this.sensorTimeline.pm25.push({x: time, y: this.sensorData.pm25});
        this.sensorTimeline.pm50.push({x: time, y: this.sensorData.pm50});
        this.sensorTimeline.pm100.push({x: time, y: this.sensorData.pm100});
        this.update();
    }

    onDetect(msg) {
        // this.draw_box(msg.bbox.center.x, msg.bbox.center.y, msg.bbox.size_x, msg.bbox.size_y, (48, 209, 88));
        const box = msg.bbox;
        box.mstamp = Date.now();
        box.color = Y_SUCCESS;
        this.boxes.push(box);
        console.log("box", box);
    }

    onCount(msg) {
        this.counts = msg;
        if (this.lastCount === -1) {
            this.lastCount = msg.total;
        }
        this.timeline.push({stamp: {secs: Date.now() / 1000, nsecs: 0}, counts: msg});
        this.rate = this.computeRate();
        this.update();
    }

    onImage(msg) {
        if (msg.img.header.seq <= this.seq) return;
        this.lastFrame = Date.now();
        this.seq = msg.img.header.seq;
        if (this.frozen) return;
        this.img.src = "data:image/jpeg;base64," + msg.img.data;
        this.onBox(msg);
        this.paint = true;

        // this.latency.network = this.last_frame - stamp_to_millis(msg.img.header.stamp);
        // this.latency.processing = stamp_to_millis(msg.header.stamp) - stamp_to_millis(msg.img.header.stamp);
        // this.latency.total = this.latency.network + this.latency.processing;
    }

    onBox(msg) {
        if (msg.boxes.length === 0) return;
        const stamp = Date.now();
        for (const box of msg.boxes) {
            box.mstamp = stamp;
            box.color = "#1ccbed";
            this.boxes.push(box);
        }
        this.lastBox = stamp;
    }

    /**
     * Load timeline from ROS
     */
    loadTimeline() {
        this.relay.callService("/sbx/timetravel/history", {}).then((res) => {
            if (res.timeline.length === 0) return;
            this.timeline = res.timeline.reverse();
            this.rate = this.computeRate();
            this.lastCount = this.timeline[this.timeline.length - 1].counts.total;
            const end = this.timeline[this.timeline.length - 1].stamp.secs * 1000;
            const start = end - 3600000 * 6;
            const step = Math.max((end - start) / 100, 10000);

            this.vehiclesOverTime.data.datasets[0].data = this.sliceTimeline(start, end, step);
            this.vehiclesOverTime.data.datasets[0].label = `Vehicles over ${(end - start) / 3600000} hours`;

            this.vehiclesOverTime.update();
        }).catch((err) => {
            this.toastError("Mission Control", `Failed to load timeline: ${err}`)
        });
    }

    /**
     * Slice timeline into data points
     * @param {number} startTime slice start time in milliseconds
     * @param {number} endTime slice end time in milliseconds
     * @param {number} step step size in milliseconds
     * @returns {Array} array of {x: Date, y: number}
     */
    sliceTimeline(startTime, endTime, step = -1) {
        const diff = endTime - startTime;
        if (step === -1) step = Math.max(diff / 60, 10000);

        let p = 0;
        while (p < this.timeline.length && stampToMillis(this.timeline[p].stamp) < startTime) p++;
        if (p === this.timeline.length) return [];

        let prev = this.timeline[p].counts.total;
        const data = [];

        for (let i = startTime; i <= endTime && p < this.timeline.length; i += step) {
            let count = 0;

            while (p < this.timeline.length && stampToMillis(this.timeline[p].stamp) < i) {
                count += this.timeline[p].counts.total - prev;
                prev = this.timeline[p].counts.total;
                p++;
            }
            data.push({x: new Date(i), y: count});
        }
        return data;
    }

    /**
     * Computes the number of vehicles per hour
     * @returns {number} the number of vehicles detected per hour
     */
    computeRate() {
        const end = this.timeline[this.timeline.length - 1].stamp.secs;
        const start = end - 3600;
        return this.sliceTimeline(start * 1000, end * 1000).reduce((acc, cur) => acc + cur.y, 0);
    }


    drawBox(centerX, centerY, sizeX, sizeY, color = Y_BLUE) {
        // this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        const x = centerX - sizeX / 2;
        const y = centerY - sizeY / 2;
        // console.log(x, y, size_x, size_y);
        this.ctx.lineWidth = 4;
        this.ctx.strokeStyle = color;
        this.ctx.strokeRect(centerX - sizeX / 2, centerY - sizeY / 2, sizeX, sizeY);
        // this.ctx.fillRect(center_x - size_x / 2, center_y - size_y / 2, size_x, size_y);
        // this.ctx.stroke();
    }

    losScreen() {
        document.getElementById("los").style.display = "grid";
        this.los = true;
        // console.log("los");
    }

    losScreenOff() {
        document.getElementById("los").style.display = "none";
        this.los = false;
        // console.log("los off");
    }

    navIn() {
        document.querySelectorAll(".nav-float").forEach((el) => {
            el.classList.remove("nav-out");
            el.classList.add("nav-in");
        });
    }

    navOut() {
        document.querySelectorAll(".nav-float").forEach((el) => {
            el.classList.remove("nav-in");
            el.classList.add("nav-out");
        });
    }

    toggleFreeze() {
        this.frozen = !this.frozen;
        if (this.frozen) {
            this.toastInfo("Mission Control", "Freeze");
        } else {
            this.toastInfo("Mission Control", "Unfreeze");
        }
    }

    /**
     * generates animation frame for video rendering
     */
    getFrame() {
        if (!this.relay.connected) return this.reconnect("Connection lost!");

        const now = Date.now();
        if (!this.frozen) {
            if (!this.los) {
                if (now - this.lastFrame > 5000 && !this.paint) {
                    this.boxes = [];
                    this.losScreen();
                    this.toastWarning("Mission Control", "No frames received");
                } else if (this.boxes.length > 0 || now - this.lastBox > 100) {
                    this.paint = false;
                    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

                    if (this.boxes.length > 0) {

                        const newBoxes = [];

                        for (const box of this.boxes) {
                            if (now - box.mstamp > 50) {
                                continue;
                            }
                            newBoxes.push(box);
                            this.drawBox(box.center.x, box.center.y, box.size_x, box.size_y, box.color);
                        }
                        this.boxes = newBoxes;
                    }
                }
            } else if (this.paint) {
                this.losScreenOff();
            }
        }
        window.requestAnimationFrame(this.getFrame.bind(this));
    }


    /**
     * Updates various UI elements
     */
    update() {
        // uncomment as you implement

        document.getElementById('total_count').innerHTML = this.rate.toFixed(2);
        document.getElementById('total_count_2').innerHTML = this.counts.total;
        document.getElementById('counts_car_2').innerHTML = this.counts.cars;
        document.getElementById('counts_bus_2').innerHTML = this.counts.buses;
        document.getElementById('counts_truck_2').innerHTML = this.counts.trucks;

        // sensor data

        document.getElementById('pm10').innerHTML = Math.round(this.sensorData.pm10 * 100) / 100;
        document.getElementById('pm25').innerHTML = Math.round(this.sensorData.pm25 * 100) / 100;
        // document.getElementById('pm50').innerHTML = this.sensor_data.pm50;
        // document.getElementById('pm100').innerHTML = this.sensor_data.pm100;
        document.getElementById('tmp').innerHTML = Math.round(this.sensorData.tmp * 100) / 100;
        document.getElementById('hum').innerHTML = Math.round(this.sensorData.hum * 100) / 100;
        document.getElementById('co2').innerHTML = Math.round(this.sensorData.co2 * 100) / 100;

        this.numberOfVehicles.data.datasets.forEach((dataset) => {
            dataset.data = [this.counts.cars, this.counts.trucks, this.counts.buses, this.counts.total - this.counts.cars - this.counts.trucks - this.counts.buses];
        });

        this.numberOfVehicles.update();
    }

    updateGraphs() {
        // setup
        let date = new Date()
        const nhours = this.graphViews[this.graphView];
        const start = date.getTime() - 3600000 * nhours;
        const end = date.getTime();
        const diff = end - start;

        // Update chart [VEHICLES OVER TIME]
        this.vehiclesOverTime.data.datasets[0].data = this.sliceTimeline(start, end);
        this.vehiclesOverTime.data.datasets[0].label = `Vehicles per ${Math.round(Math.max(diff / 60000, 10))} seconds`;
        this.vehiclesOverTime.update();

        // Update chart [temperature]
        this.temperatureChart.data.datasets[0].data = this.sensorTimeline.tmp.filter(d => d.x > start);
        this.temperatureChart.data.datasets[0].label = `Temperature [°C] (last ${nhours} hours)`;
        this.temperatureChart.update();

        // Update chart [humidity]
        this.humidityChart.data.datasets[0].data = this.sensorTimeline.hum.filter(d => d.x > start);
        this.humidityChart.data.datasets[0].label = `Humidity [%] (last ${nhours} hours)`;
        this.humidityChart.update();

        // Update chart [pm10]
        this.pm10Chart.data.datasets[0].data = this.sensorTimeline.pm10.filter(d => d.x > start);
        this.pm10Chart.data.datasets[0].label = `PM10 [µg/m³] (last ${nhours} hours)`;
        this.pm10Chart.update();

        // Update chart [pm25]
        this.pm25Chart.data.datasets[0].data = this.sensorTimeline.pm25.filter(d => d.x > start);
        this.pm25Chart.data.datasets[0].label = `PM2.5 [µg/m³] (last ${nhours} hours)`;
        this.pm25Chart.update();

        // Update chart [co2]
        this.co2Chart.data.datasets[0].data = this.sensorTimeline.co2.filter(d => d.x > start);
        this.co2Chart.data.datasets[0].label = `CO2 [ppm] (last ${nhours} hours)`;
        this.co2Chart.update();
    }

    // utilities

    /**
     * Displays a temporary notification
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


const sensorLive = new SensorLive();

window.addEventListener('keydown', (e) => {
    if (e.key === 'Tab' && sensorLive.connected) {
        e.preventDefault();
        if (triggerTabList[0].classList.contains('active')) {
            tabs[1].show();
        } else if (triggerTabList[1].classList.contains('active')) {
            tabs[0].show();
        }
    } else if (e.key === 'Escape') {
        // tabs[0].show();
        sensorLive.toastElement.hide();
    } else if (e.key === ' ') {
        sensorLive.toggleFreeze();
    }
});

