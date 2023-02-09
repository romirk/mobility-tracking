/**
 * Bridge between UI and ROS
 */
class Relay {
    url;
    ros;
    listeners = {};
    services = {};
    connected = false;

    constructor(host, port) {
        this.url = "ws://" + host + ":" + port;
    }

    /**
     * @deprecated print log
     * @param topic
     * @param string
     */
    p(topic, string) {
        console.log(`[${topic}] ${string}`);
    }

    /**
     * @deprecated print warning
     * @param str
     */
    w(str) {
        console.warn(str);
    }

    /**
     * @deprecated print error
     * @param str
     */
    e(str) {
        console.error(str);
    }

    /**
     * Listen to a ROS topic and process using callback.
     * @param {string} topic
     * @param {string} type
     * @param {function} callback
     */
    createListener(topic, type = "std_msgs/String", callback = str => console.log(topic, str)) {
        this.listeners[topic] = new ROSLIB.Topic({
            ros: this.ros, name: topic, messageType: type
        });
        this.listeners[topic].subscribe(callback);

        console.log(`Created listener for topic ${topic}`);
        if (!this.connected) console.warn("Not connected to ROS");
    }


    /**
     * Register a service.
     * @param service
     * @param type
     */
    createService(service, type) {
        this.services[service] = new ROSLIB.Service({
            ros: this.ros, name: service, serviceType: type
        });
        console.log(`Created handler for service ${service}`);

        if (!this.connected) console.warn("Not connected to ROS");

    }

    connect() {
        return new Promise((resolve, reject) => {
            console.log("connecting...");
            const ros = new ROSLIB.Ros({
                url: this.url
            });
            this.ros = ros;
            const url = this.url;

            ros.on('connection', (function () {
                console.log(`Connected to ${url}.`);
                this.connected = true;
                this.createService("/rosapi/nodes", "rosapi/Nodes");
                this.createService("/rosapi/topics", "rosapi/Topics");
                this.createService("/rosapi/services", "rosapi/Services");
                resolve();
            }).bind(this));

            ros.on('error', _ => {
                console.error(`Error connecting to websocket server at ${url}`);
                reject();
            });

            ros.on('close', (function () {
                console.log('Connection to websocket server closed.');
                this.connected = false;
                reject();
            }).bind(this));
        });
    }

    disconnect() {
        this.ros.close();
    }

    /**
     * Call the indicated service with the given arguments.
     * Service must be recognized by `Relay` (through `createService`).
     * @param {string} service
     * @param {object} args
     * @returns {Promise}
     */
    callService(service, args = {}) {
        if (!this.connected) throw "Not connected to ROS";
        if (service === undefined) throw "service undefined";

        const req = new ROSLIB.ServiceRequest(args);
        const client = this.services[service];
        console.log(client, req);
        return new Promise((resolve, reject) => {
            client.callService(req, resolve, reject);
        });
    }

    getTopics(callback) {
        this.callService("/rosapi/topics", {}).then(callback);
    }

    getServices(callback) {
        this.callService("/rosapi/services", {}).then(callback);
    }

    getNodes(callback) {
        this.callService("/rosapi/nodes", {}).then(callback);
    }

    reset() {
        this.disconnect();
        this.listeners = {};
    }

}

