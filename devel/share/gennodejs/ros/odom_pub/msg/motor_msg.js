// Auto-generated. Do not edit!

// (in-package odom_pub.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class motor_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.driver_motor_01 = null;
      this.driver_motor_02 = null;
      this.driver_motor_03 = null;
      this.driver_motor_04 = null;
      this.steer_motor_01 = null;
      this.steer_motor_02 = null;
      this.steer_motor_03 = null;
      this.steer_motor_04 = null;
    }
    else {
      if (initObj.hasOwnProperty('driver_motor_01')) {
        this.driver_motor_01 = initObj.driver_motor_01
      }
      else {
        this.driver_motor_01 = 0.0;
      }
      if (initObj.hasOwnProperty('driver_motor_02')) {
        this.driver_motor_02 = initObj.driver_motor_02
      }
      else {
        this.driver_motor_02 = 0.0;
      }
      if (initObj.hasOwnProperty('driver_motor_03')) {
        this.driver_motor_03 = initObj.driver_motor_03
      }
      else {
        this.driver_motor_03 = 0.0;
      }
      if (initObj.hasOwnProperty('driver_motor_04')) {
        this.driver_motor_04 = initObj.driver_motor_04
      }
      else {
        this.driver_motor_04 = 0.0;
      }
      if (initObj.hasOwnProperty('steer_motor_01')) {
        this.steer_motor_01 = initObj.steer_motor_01
      }
      else {
        this.steer_motor_01 = 0.0;
      }
      if (initObj.hasOwnProperty('steer_motor_02')) {
        this.steer_motor_02 = initObj.steer_motor_02
      }
      else {
        this.steer_motor_02 = 0.0;
      }
      if (initObj.hasOwnProperty('steer_motor_03')) {
        this.steer_motor_03 = initObj.steer_motor_03
      }
      else {
        this.steer_motor_03 = 0.0;
      }
      if (initObj.hasOwnProperty('steer_motor_04')) {
        this.steer_motor_04 = initObj.steer_motor_04
      }
      else {
        this.steer_motor_04 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type motor_msg
    // Serialize message field [driver_motor_01]
    bufferOffset = _serializer.float64(obj.driver_motor_01, buffer, bufferOffset);
    // Serialize message field [driver_motor_02]
    bufferOffset = _serializer.float64(obj.driver_motor_02, buffer, bufferOffset);
    // Serialize message field [driver_motor_03]
    bufferOffset = _serializer.float64(obj.driver_motor_03, buffer, bufferOffset);
    // Serialize message field [driver_motor_04]
    bufferOffset = _serializer.float64(obj.driver_motor_04, buffer, bufferOffset);
    // Serialize message field [steer_motor_01]
    bufferOffset = _serializer.float64(obj.steer_motor_01, buffer, bufferOffset);
    // Serialize message field [steer_motor_02]
    bufferOffset = _serializer.float64(obj.steer_motor_02, buffer, bufferOffset);
    // Serialize message field [steer_motor_03]
    bufferOffset = _serializer.float64(obj.steer_motor_03, buffer, bufferOffset);
    // Serialize message field [steer_motor_04]
    bufferOffset = _serializer.float64(obj.steer_motor_04, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type motor_msg
    let len;
    let data = new motor_msg(null);
    // Deserialize message field [driver_motor_01]
    data.driver_motor_01 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [driver_motor_02]
    data.driver_motor_02 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [driver_motor_03]
    data.driver_motor_03 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [driver_motor_04]
    data.driver_motor_04 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [steer_motor_01]
    data.steer_motor_01 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [steer_motor_02]
    data.steer_motor_02 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [steer_motor_03]
    data.steer_motor_03 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [steer_motor_04]
    data.steer_motor_04 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'odom_pub/motor_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0e4c0fdbf8c8204c136aa0e02d2bf289';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 driver_motor_01     
    float64 driver_motor_02
    float64 driver_motor_03
    float64 driver_motor_04
    
    float64 steer_motor_01
    float64 steer_motor_02
    float64 steer_motor_03
    float64 steer_motor_04
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new motor_msg(null);
    if (msg.driver_motor_01 !== undefined) {
      resolved.driver_motor_01 = msg.driver_motor_01;
    }
    else {
      resolved.driver_motor_01 = 0.0
    }

    if (msg.driver_motor_02 !== undefined) {
      resolved.driver_motor_02 = msg.driver_motor_02;
    }
    else {
      resolved.driver_motor_02 = 0.0
    }

    if (msg.driver_motor_03 !== undefined) {
      resolved.driver_motor_03 = msg.driver_motor_03;
    }
    else {
      resolved.driver_motor_03 = 0.0
    }

    if (msg.driver_motor_04 !== undefined) {
      resolved.driver_motor_04 = msg.driver_motor_04;
    }
    else {
      resolved.driver_motor_04 = 0.0
    }

    if (msg.steer_motor_01 !== undefined) {
      resolved.steer_motor_01 = msg.steer_motor_01;
    }
    else {
      resolved.steer_motor_01 = 0.0
    }

    if (msg.steer_motor_02 !== undefined) {
      resolved.steer_motor_02 = msg.steer_motor_02;
    }
    else {
      resolved.steer_motor_02 = 0.0
    }

    if (msg.steer_motor_03 !== undefined) {
      resolved.steer_motor_03 = msg.steer_motor_03;
    }
    else {
      resolved.steer_motor_03 = 0.0
    }

    if (msg.steer_motor_04 !== undefined) {
      resolved.steer_motor_04 = msg.steer_motor_04;
    }
    else {
      resolved.steer_motor_04 = 0.0
    }

    return resolved;
    }
};

module.exports = motor_msg;
