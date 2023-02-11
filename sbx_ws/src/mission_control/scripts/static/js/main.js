const screen = document.getElementById('live');

// get colors from css
const Y_BLUE = getComputedStyle(document.documentElement).getPropertyValue('--y-blue');
const Y_DBLUE = getComputedStyle(document.documentElement).getPropertyValue('--y-dblue');
const Y_SUCCESS = getComputedStyle(document.documentElement).getPropertyValue('--y-success');
const Y_WARNING = getComputedStyle(document.documentElement).getPropertyValue('--y-warning');
const Y_DANGER = getComputedStyle(document.documentElement).getPropertyValue('--y-danger');


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
        this.sensorData = {
            pm10: 0, pm25: 0, pm50: 0, pm100: 0, tmp: 0, hum: 0, co2: 0
        };

        this.lastCount = -1;
        this.timeline = [];
        this.boxes = []
        this.paint = true;
        this.lastBox = 0;
        this.lastFrame = 0;
        this.seq = 0;
        this.los = false;
        this.frozen = false;
        this.latency = { network: 0, processing: 0, total: 0 };

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
        this.vehiclesOverTime.data.datasets[0].label = 'vehicles over time';
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
        this.toastInfo("Mission Control", "Connecting...");
        this.relay.connect()
            .then(this.rosSetup.bind(this))
            .then(() => sleep(2000)) // wait for connection to stabilize
            .then(this.setup.bind(this))
            .catch(() => this.reconnect("Failed to connect"));
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
        this.connected = false;
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
        this.relay.createServiceClient("/sbx/timeline", "mission_control/Timeline");
    }

    setup() {
        this.connected = true;
        this.toastSuccess("Mission Control", "Connected");
        this.losScreenOff();
        this.navIn();
        this.loadTimeline();

        setInterval(() => {
            this.graphView = (this.graphView + 1) % this.graphViews.length;
            this.updateGraphs()
        }, 10000);
        window.requestAnimationFrame(this.getFrame.bind(this));
    }

    onSensor(msg) {
        this.sensorData = msg;
        // this.log();
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
        this.timeline.push({ stamp: { secs: Date.now() / 1000, nsecs: 0 }, counts: msg });
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

    loadTimeline() {
        this.relay.callService("/sbx/timeline", {}).then((res) => {
            if (res.timeline.length === 0) return;
            this.timeline = res.timeline.reverse();
            const end = this.timeline[this.timeline.length - 1].stamp.secs * 1000;
            const start = end - 3600000 * 6;
            const step = Math.max((end - start) / 100, 10000);

            console.log(start, end, step, end - start);

            this.vehiclesOverTime.data.datasets[0].data = this.sliceTimeline(start, end, step);
            this.vehiclesOverTime.update();
        }).catch((err) => {
            this.toastError("Mission Control", `Failed to load timeline: ${err}`)
        });
    }

    sliceTimeline(startTime, endTime, step = -1) {
        const diff = endTime - startTime;
        if (step === -1) step = Math.max(diff / 100, 10000);

        console.log(startTime, endTime, diff, step);
        let p = 0;
        while (p < this.timeline.length && stampToMillis(this.timeline[p].stamp) < startTime)
            p++;
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
            data.push({ x: new Date(i), y: count });
        }
        return data;
    }

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
        }
        else {
            this.toastInfo("Mission Control", "Unfreeze");
        }
    }

    getFrame() {
        if (!this.relay.connected) return this.reconnect("Connection lost!");

        const now = Date.now();
        if (!this.frozen) {
            if (!this.los) {
                if (now - this.lastFrame > 5000 && !this.paint) {
                    this.boxes = [];
                    this.losScreen();
                    this.toastWarning("Mission Control", "No frames received");
                }
                else if (this.boxes.length > 0 || now - this.lastBox > 100) {
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
            }
            else if (this.paint) {
                this.losScreenOff();
            }
        }
        window.requestAnimationFrame(this.getFrame.bind(this));
    }


    update() {
        // uncomment as you implement

        document.getElementById('total_count').innerHTML = this.computeRate().toFixed(2);
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
        // Update chart [VEHICLES OVER TIME]
        let date = new Date()

        const nhours = this.graphViews[this.graphView];
        this.vehiclesOverTime.data.datasets[0].data = this.sliceTimeline(date.getTime() - 3600000 * nhours, date.getTime());
        this.vehiclesOverTime.data.datasets[0].label = `Vehicles (last ${nhours} hours)`;
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

const triggerTabList = document.querySelectorAll('#selector button')
const tabs = [];

triggerTabList.forEach(triggerEl => {
    const tabTrigger = new bootstrap.Tab(triggerEl)

    tabs.push(tabTrigger);
});

const sensorLive = new SensorLive();

window.addEventListener('keydown', (e) => {
    if (e.key === 'Tab' && sensorLive.connected) {
        e.preventDefault();
        if (triggerTabList[0].classList.contains('active')) {
            tabs[1].show();
        }
        else if (triggerTabList[1].classList.contains('active')) {
            tabs[0].show();
        }
    }
    else if (e.key === 'Escape') {
        // tabs[0].show();
        sensorLive.toastElement.hide();
    }
    else if (e.key === ' ') {
        sensorLive.toggleFreeze();
    }
});

