// Auto-generated. Do not edit!

// (in-package dynamo_planner.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class custom_states_msgs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.yaw = null;
      this.controlX = null;
      this.controlY = null;
      this.controlYAW = null;
      this.duration = null;
      this.pre_x = null;
      this.pre_y = null;
      this.pre_yaw = null;
      this.flag = null;
      this.tf_flag = null;
      this.diff_x = null;
      this.diff_y = null;
      this.diff_yaw = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('controlX')) {
        this.controlX = initObj.controlX
      }
      else {
        this.controlX = 0.0;
      }
      if (initObj.hasOwnProperty('controlY')) {
        this.controlY = initObj.controlY
      }
      else {
        this.controlY = 0.0;
      }
      if (initObj.hasOwnProperty('controlYAW')) {
        this.controlYAW = initObj.controlYAW
      }
      else {
        this.controlYAW = 0.0;
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0.0;
      }
      if (initObj.hasOwnProperty('pre_x')) {
        this.pre_x = initObj.pre_x
      }
      else {
        this.pre_x = 0.0;
      }
      if (initObj.hasOwnProperty('pre_y')) {
        this.pre_y = initObj.pre_y
      }
      else {
        this.pre_y = 0.0;
      }
      if (initObj.hasOwnProperty('pre_yaw')) {
        this.pre_yaw = initObj.pre_yaw
      }
      else {
        this.pre_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = false;
      }
      if (initObj.hasOwnProperty('tf_flag')) {
        this.tf_flag = initObj.tf_flag
      }
      else {
        this.tf_flag = false;
      }
      if (initObj.hasOwnProperty('diff_x')) {
        this.diff_x = initObj.diff_x
      }
      else {
        this.diff_x = false;
      }
      if (initObj.hasOwnProperty('diff_y')) {
        this.diff_y = initObj.diff_y
      }
      else {
        this.diff_y = false;
      }
      if (initObj.hasOwnProperty('diff_yaw')) {
        this.diff_yaw = initObj.diff_yaw
      }
      else {
        this.diff_yaw = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type custom_states_msgs
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [controlX]
    bufferOffset = _serializer.float64(obj.controlX, buffer, bufferOffset);
    // Serialize message field [controlY]
    bufferOffset = _serializer.float64(obj.controlY, buffer, bufferOffset);
    // Serialize message field [controlYAW]
    bufferOffset = _serializer.float64(obj.controlYAW, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float64(obj.duration, buffer, bufferOffset);
    // Serialize message field [pre_x]
    bufferOffset = _serializer.float64(obj.pre_x, buffer, bufferOffset);
    // Serialize message field [pre_y]
    bufferOffset = _serializer.float64(obj.pre_y, buffer, bufferOffset);
    // Serialize message field [pre_yaw]
    bufferOffset = _serializer.float64(obj.pre_yaw, buffer, bufferOffset);
    // Serialize message field [flag]
    bufferOffset = _serializer.bool(obj.flag, buffer, bufferOffset);
    // Serialize message field [tf_flag]
    bufferOffset = _serializer.bool(obj.tf_flag, buffer, bufferOffset);
    // Serialize message field [diff_x]
    bufferOffset = _serializer.bool(obj.diff_x, buffer, bufferOffset);
    // Serialize message field [diff_y]
    bufferOffset = _serializer.bool(obj.diff_y, buffer, bufferOffset);
    // Serialize message field [diff_yaw]
    bufferOffset = _serializer.bool(obj.diff_yaw, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type custom_states_msgs
    let len;
    let data = new custom_states_msgs(null);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [controlX]
    data.controlX = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [controlY]
    data.controlY = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [controlYAW]
    data.controlYAW = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pre_x]
    data.pre_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pre_y]
    data.pre_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pre_yaw]
    data.pre_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [flag]
    data.flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tf_flag]
    data.tf_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [diff_x]
    data.diff_x = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [diff_y]
    data.diff_y = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [diff_yaw]
    data.diff_yaw = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 85;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dynamo_planner/custom_states_msgs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3b52bf44a08210bd7d5f8e100b5f756a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # double type x y yaw control duration
    float64 x
    float64 y
    float64 yaw
    
    float64 controlX
    float64 controlY
    float64 controlYAW
    
    float64 duration
    
    float64 pre_x
    float64 pre_y
    float64 pre_yaw
    
    bool flag
    bool tf_flag
    
    bool diff_x
    bool diff_y
    bool diff_yaw
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new custom_states_msgs(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.controlX !== undefined) {
      resolved.controlX = msg.controlX;
    }
    else {
      resolved.controlX = 0.0
    }

    if (msg.controlY !== undefined) {
      resolved.controlY = msg.controlY;
    }
    else {
      resolved.controlY = 0.0
    }

    if (msg.controlYAW !== undefined) {
      resolved.controlYAW = msg.controlYAW;
    }
    else {
      resolved.controlYAW = 0.0
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0.0
    }

    if (msg.pre_x !== undefined) {
      resolved.pre_x = msg.pre_x;
    }
    else {
      resolved.pre_x = 0.0
    }

    if (msg.pre_y !== undefined) {
      resolved.pre_y = msg.pre_y;
    }
    else {
      resolved.pre_y = 0.0
    }

    if (msg.pre_yaw !== undefined) {
      resolved.pre_yaw = msg.pre_yaw;
    }
    else {
      resolved.pre_yaw = 0.0
    }

    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = false
    }

    if (msg.tf_flag !== undefined) {
      resolved.tf_flag = msg.tf_flag;
    }
    else {
      resolved.tf_flag = false
    }

    if (msg.diff_x !== undefined) {
      resolved.diff_x = msg.diff_x;
    }
    else {
      resolved.diff_x = false
    }

    if (msg.diff_y !== undefined) {
      resolved.diff_y = msg.diff_y;
    }
    else {
      resolved.diff_y = false
    }

    if (msg.diff_yaw !== undefined) {
      resolved.diff_yaw = msg.diff_yaw;
    }
    else {
      resolved.diff_yaw = false
    }

    return resolved;
    }
};

module.exports = custom_states_msgs;
