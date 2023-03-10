// Auto-generated. Do not edit!

// (in-package ur3_move.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class movingObjectPositionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.send = null;
    }
    else {
      if (initObj.hasOwnProperty('send')) {
        this.send = initObj.send
      }
      else {
        this.send = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type movingObjectPositionRequest
    // Serialize message field [send]
    bufferOffset = _serializer.bool(obj.send, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type movingObjectPositionRequest
    let len;
    let data = new movingObjectPositionRequest(null);
    // Deserialize message field [send]
    data.send = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur3_move/movingObjectPositionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87208eaee51dca4c39db55d7a40b357d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool send
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new movingObjectPositionRequest(null);
    if (msg.send !== undefined) {
      resolved.send = msg.send;
    }
    else {
      resolved.send = false
    }

    return resolved;
    }
};

class movingObjectPositionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.receive = null;
    }
    else {
      if (initObj.hasOwnProperty('receive')) {
        this.receive = initObj.receive
      }
      else {
        this.receive = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type movingObjectPositionResponse
    // Serialize message field [receive]
    bufferOffset = _serializer.bool(obj.receive, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type movingObjectPositionResponse
    let len;
    let data = new movingObjectPositionResponse(null);
    // Deserialize message field [receive]
    data.receive = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur3_move/movingObjectPositionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7015dad2877ea1de0b9f990426d1de00';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool receive
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new movingObjectPositionResponse(null);
    if (msg.receive !== undefined) {
      resolved.receive = msg.receive;
    }
    else {
      resolved.receive = false
    }

    return resolved;
    }
};

module.exports = {
  Request: movingObjectPositionRequest,
  Response: movingObjectPositionResponse,
  md5sum() { return '05475c87257688fa5ef65958faf092d7'; },
  datatype() { return 'ur3_move/movingObjectPosition'; }
};
