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

class data_gen_msgs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.flag = null;
      this.filename = null;
      this.num_data = null;
    }
    else {
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = false;
      }
      if (initObj.hasOwnProperty('filename')) {
        this.filename = initObj.filename
      }
      else {
        this.filename = '';
      }
      if (initObj.hasOwnProperty('num_data')) {
        this.num_data = initObj.num_data
      }
      else {
        this.num_data = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type data_gen_msgs
    // Serialize message field [flag]
    bufferOffset = _serializer.bool(obj.flag, buffer, bufferOffset);
    // Serialize message field [filename]
    bufferOffset = _serializer.string(obj.filename, buffer, bufferOffset);
    // Serialize message field [num_data]
    bufferOffset = _serializer.float64(obj.num_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type data_gen_msgs
    let len;
    let data = new data_gen_msgs(null);
    // Deserialize message field [flag]
    data.flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [filename]
    data.filename = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [num_data]
    data.num_data = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.filename.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dynamo_planner/data_gen_msgs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5217da41250023fd463ea99124c27fdc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # bool flag string filename float64 num_data
    bool flag
    string filename
    float64 num_data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new data_gen_msgs(null);
    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = false
    }

    if (msg.filename !== undefined) {
      resolved.filename = msg.filename;
    }
    else {
      resolved.filename = ''
    }

    if (msg.num_data !== undefined) {
      resolved.num_data = msg.num_data;
    }
    else {
      resolved.num_data = 0.0
    }

    return resolved;
    }
};

module.exports = data_gen_msgs;
