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

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class varObjectsPositionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.flag = null;
    }
    else {
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type varObjectsPositionRequest
    // Serialize message field [flag]
    bufferOffset = _serializer.bool(obj.flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type varObjectsPositionRequest
    let len;
    let data = new varObjectsPositionRequest(null);
    // Deserialize message field [flag]
    data.flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur3_move/varObjectsPositionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '24842bc754e0f5cc982338eca1269251';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool flag
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new varObjectsPositionRequest(null);
    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = false
    }

    return resolved;
    }
};

class varObjectsPositionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.targets_pose = null;
      this.angles = null;
      this.names = null;
    }
    else {
      if (initObj.hasOwnProperty('targets_pose')) {
        this.targets_pose = initObj.targets_pose
      }
      else {
        this.targets_pose = [];
      }
      if (initObj.hasOwnProperty('angles')) {
        this.angles = initObj.angles
      }
      else {
        this.angles = [];
      }
      if (initObj.hasOwnProperty('names')) {
        this.names = initObj.names
      }
      else {
        this.names = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type varObjectsPositionResponse
    // Serialize message field [targets_pose]
    // Serialize the length for message field [targets_pose]
    bufferOffset = _serializer.uint32(obj.targets_pose.length, buffer, bufferOffset);
    obj.targets_pose.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [angles]
    bufferOffset = _arraySerializer.float64(obj.angles, buffer, bufferOffset, null);
    // Serialize message field [names]
    bufferOffset = _arraySerializer.string(obj.names, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type varObjectsPositionResponse
    let len;
    let data = new varObjectsPositionResponse(null);
    // Deserialize message field [targets_pose]
    // Deserialize array length for message field [targets_pose]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.targets_pose = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.targets_pose[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [angles]
    data.angles = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [names]
    data.names = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 56 * object.targets_pose.length;
    length += 8 * object.angles.length;
    object.names.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur3_move/varObjectsPositionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '79247432a0172fe9912f0f1afeeeca54';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose[] targets_pose
    float64[] angles
    string[] names
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new varObjectsPositionResponse(null);
    if (msg.targets_pose !== undefined) {
      resolved.targets_pose = new Array(msg.targets_pose.length);
      for (let i = 0; i < resolved.targets_pose.length; ++i) {
        resolved.targets_pose[i] = geometry_msgs.msg.Pose.Resolve(msg.targets_pose[i]);
      }
    }
    else {
      resolved.targets_pose = []
    }

    if (msg.angles !== undefined) {
      resolved.angles = msg.angles;
    }
    else {
      resolved.angles = []
    }

    if (msg.names !== undefined) {
      resolved.names = msg.names;
    }
    else {
      resolved.names = []
    }

    return resolved;
    }
};

module.exports = {
  Request: varObjectsPositionRequest,
  Response: varObjectsPositionResponse,
  md5sum() { return '471fff188bc2b994ff747dcd01944fb6'; },
  datatype() { return 'ur3_move/varObjectsPosition'; }
};
