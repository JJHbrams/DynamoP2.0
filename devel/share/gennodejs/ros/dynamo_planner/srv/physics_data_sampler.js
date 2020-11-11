// Auto-generated. Do not edit!

// (in-package dynamo_planner.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class physics_data_samplerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.NUM_STEP = null;
    }
    else {
      if (initObj.hasOwnProperty('NUM_STEP')) {
        this.NUM_STEP = initObj.NUM_STEP
      }
      else {
        this.NUM_STEP = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type physics_data_samplerRequest
    // Serialize message field [NUM_STEP]
    bufferOffset = _serializer.int64(obj.NUM_STEP, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type physics_data_samplerRequest
    let len;
    let data = new physics_data_samplerRequest(null);
    // Deserialize message field [NUM_STEP]
    data.NUM_STEP = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamo_planner/physics_data_samplerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '521a14b05672e26237fdd5647755858f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int64 NUM_STEP
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new physics_data_samplerRequest(null);
    if (msg.NUM_STEP !== undefined) {
      resolved.NUM_STEP = msg.NUM_STEP;
    }
    else {
      resolved.NUM_STEP = 0
    }

    return resolved;
    }
};

class physics_data_samplerResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type physics_data_samplerResponse
    // Serialize message field [result]
    bufferOffset = _serializer.bool(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type physics_data_samplerResponse
    let len;
    let data = new physics_data_samplerResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamo_planner/physics_data_samplerResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eb13ac1f1354ccecb7941ee8fa2192e8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    bool result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new physics_data_samplerResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = false
    }

    return resolved;
    }
};

module.exports = {
  Request: physics_data_samplerRequest,
  Response: physics_data_samplerResponse,
  md5sum() { return 'd13b09ac032499e03ad2334da1a598d0'; },
  datatype() { return 'dynamo_planner/physics_data_sampler'; }
};
