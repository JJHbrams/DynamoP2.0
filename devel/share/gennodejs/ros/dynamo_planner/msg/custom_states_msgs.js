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
      this.controlA = null;
      this.controlB = null;
      this.control = null;
      this.controlYAW = null;
      this.duration = null;
      this.pre_x = null;
      this.pre_y = null;
      this.pre_yaw = null;
      this.pre_controlA = null;
      this.pre_controlB = null;
      this.flag = null;
      this.sub_flag = null;
      this.index = null;
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
      if (initObj.hasOwnProperty('controlA')) {
        this.controlA = initObj.controlA
      }
      else {
        this.controlA = 0.0;
      }
      if (initObj.hasOwnProperty('controlB')) {
        this.controlB = initObj.controlB
      }
      else {
        this.controlB = 0.0;
      }
      if (initObj.hasOwnProperty('control')) {
        this.control = initObj.control
      }
      else {
        this.control = 0.0;
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
      if (initObj.hasOwnProperty('pre_controlA')) {
        this.pre_controlA = initObj.pre_controlA
      }
      else {
        this.pre_controlA = 0.0;
      }
      if (initObj.hasOwnProperty('pre_controlB')) {
        this.pre_controlB = initObj.pre_controlB
      }
      else {
        this.pre_controlB = 0.0;
      }
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = false;
      }
      if (initObj.hasOwnProperty('sub_flag')) {
        this.sub_flag = initObj.sub_flag
      }
      else {
        this.sub_flag = false;
      }
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = 0;
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
    // Serialize message field [controlA]
    bufferOffset = _serializer.float64(obj.controlA, buffer, bufferOffset);
    // Serialize message field [controlB]
    bufferOffset = _serializer.float64(obj.controlB, buffer, bufferOffset);
    // Serialize message field [control]
    bufferOffset = _serializer.float64(obj.control, buffer, bufferOffset);
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
    // Serialize message field [pre_controlA]
    bufferOffset = _serializer.float64(obj.pre_controlA, buffer, bufferOffset);
    // Serialize message field [pre_controlB]
    bufferOffset = _serializer.float64(obj.pre_controlB, buffer, bufferOffset);
    // Serialize message field [flag]
    bufferOffset = _serializer.bool(obj.flag, buffer, bufferOffset);
    // Serialize message field [sub_flag]
    bufferOffset = _serializer.bool(obj.sub_flag, buffer, bufferOffset);
    // Serialize message field [index]
    bufferOffset = _serializer.int8(obj.index, buffer, bufferOffset);
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
    // Deserialize message field [controlA]
    data.controlA = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [controlB]
    data.controlB = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [control]
    data.control = _deserializer.float64(buffer, bufferOffset);
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
    // Deserialize message field [pre_controlA]
    data.pre_controlA = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pre_controlB]
    data.pre_controlB = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [flag]
    data.flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [sub_flag]
    data.sub_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [index]
    data.index = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 107;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dynamo_planner/custom_states_msgs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '687ab8d67518730041fc3c6e12a166b7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # double type x y yaw control duration
    float64 x
    float64 y
    float64 yaw
    
    float64 controlA
    float64 controlB
    float64 control
    float64 controlYAW
    
    float64 duration
    
    float64 pre_x
    float64 pre_y
    float64 pre_yaw
    float64 pre_controlA
    float64 pre_controlB
    
    bool flag
    bool sub_flag
    
    int8 index
    
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

    if (msg.controlA !== undefined) {
      resolved.controlA = msg.controlA;
    }
    else {
      resolved.controlA = 0.0
    }

    if (msg.controlB !== undefined) {
      resolved.controlB = msg.controlB;
    }
    else {
      resolved.controlB = 0.0
    }

    if (msg.control !== undefined) {
      resolved.control = msg.control;
    }
    else {
      resolved.control = 0.0
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

    if (msg.pre_controlA !== undefined) {
      resolved.pre_controlA = msg.pre_controlA;
    }
    else {
      resolved.pre_controlA = 0.0
    }

    if (msg.pre_controlB !== undefined) {
      resolved.pre_controlB = msg.pre_controlB;
    }
    else {
      resolved.pre_controlB = 0.0
    }

    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = false
    }

    if (msg.sub_flag !== undefined) {
      resolved.sub_flag = msg.sub_flag;
    }
    else {
      resolved.sub_flag = false
    }

    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = 0
    }

    return resolved;
    }
};

module.exports = custom_states_msgs;
