// Auto-generated. Do not edit!

// (in-package sensorbox.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class AQI {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pm10 = null;
      this.pm25 = null;
      this.pm50 = null;
      this.pm100 = null;
      this.tmp = null;
      this.hum = null;
      this.co2 = null;
    }
    else {
      if (initObj.hasOwnProperty('pm10')) {
        this.pm10 = initObj.pm10
      }
      else {
        this.pm10 = 0.0;
      }
      if (initObj.hasOwnProperty('pm25')) {
        this.pm25 = initObj.pm25
      }
      else {
        this.pm25 = 0.0;
      }
      if (initObj.hasOwnProperty('pm50')) {
        this.pm50 = initObj.pm50
      }
      else {
        this.pm50 = 0.0;
      }
      if (initObj.hasOwnProperty('pm100')) {
        this.pm100 = initObj.pm100
      }
      else {
        this.pm100 = 0.0;
      }
      if (initObj.hasOwnProperty('tmp')) {
        this.tmp = initObj.tmp
      }
      else {
        this.tmp = 0.0;
      }
      if (initObj.hasOwnProperty('hum')) {
        this.hum = initObj.hum
      }
      else {
        this.hum = 0.0;
      }
      if (initObj.hasOwnProperty('co2')) {
        this.co2 = initObj.co2
      }
      else {
        this.co2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AQI
    // Serialize message field [pm10]
    bufferOffset = _serializer.float32(obj.pm10, buffer, bufferOffset);
    // Serialize message field [pm25]
    bufferOffset = _serializer.float32(obj.pm25, buffer, bufferOffset);
    // Serialize message field [pm50]
    bufferOffset = _serializer.float32(obj.pm50, buffer, bufferOffset);
    // Serialize message field [pm100]
    bufferOffset = _serializer.float32(obj.pm100, buffer, bufferOffset);
    // Serialize message field [tmp]
    bufferOffset = _serializer.float32(obj.tmp, buffer, bufferOffset);
    // Serialize message field [hum]
    bufferOffset = _serializer.float32(obj.hum, buffer, bufferOffset);
    // Serialize message field [co2]
    bufferOffset = _serializer.float32(obj.co2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AQI
    let len;
    let data = new AQI(null);
    // Deserialize message field [pm10]
    data.pm10 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pm25]
    data.pm25 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pm50]
    data.pm50 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pm100]
    data.pm100 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tmp]
    data.tmp = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [hum]
    data.hum = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [co2]
    data.co2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sensorbox/AQI';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '40b6414f047f4614963c31c7b66060cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 pm10
    float32 pm25
    float32 pm50
    float32 pm100
    float32 tmp
    float32 hum
    float32 co2
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AQI(null);
    if (msg.pm10 !== undefined) {
      resolved.pm10 = msg.pm10;
    }
    else {
      resolved.pm10 = 0.0
    }

    if (msg.pm25 !== undefined) {
      resolved.pm25 = msg.pm25;
    }
    else {
      resolved.pm25 = 0.0
    }

    if (msg.pm50 !== undefined) {
      resolved.pm50 = msg.pm50;
    }
    else {
      resolved.pm50 = 0.0
    }

    if (msg.pm100 !== undefined) {
      resolved.pm100 = msg.pm100;
    }
    else {
      resolved.pm100 = 0.0
    }

    if (msg.tmp !== undefined) {
      resolved.tmp = msg.tmp;
    }
    else {
      resolved.tmp = 0.0
    }

    if (msg.hum !== undefined) {
      resolved.hum = msg.hum;
    }
    else {
      resolved.hum = 0.0
    }

    if (msg.co2 !== undefined) {
      resolved.co2 = msg.co2;
    }
    else {
      resolved.co2 = 0.0
    }

    return resolved;
    }
};

module.exports = AQI;
