// Auto-generated. Do not edit!

// (in-package my_class_pkg.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class MyServiceMsgRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MyServiceMsgRequest
    // Serialize message field [input]
    bufferOffset = _serializer.int64(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MyServiceMsgRequest
    let len;
    let data = new MyServiceMsgRequest(null);
    // Deserialize message field [input]
    data.input = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'my_class_pkg/MyServiceMsgRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'efb1911a9ea9163151d70e4586fa0801';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 input # 查询
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MyServiceMsgRequest(null);
    if (msg.input !== undefined) {
      resolved.input = msg.input;
    }
    else {
      resolved.input = 0
    }

    return resolved;
    }
};

class MyServiceMsgResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MyServiceMsgResponse
    // Serialize message field [output]
    bufferOffset = _serializer.int64(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MyServiceMsgResponse
    let len;
    let data = new MyServiceMsgResponse(null);
    // Deserialize message field [output]
    data.output = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'my_class_pkg/MyServiceMsgResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eaf29e6e649dbd18239f4c3f12579de7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 output # 回复
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MyServiceMsgResponse(null);
    if (msg.output !== undefined) {
      resolved.output = msg.output;
    }
    else {
      resolved.output = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: MyServiceMsgRequest,
  Response: MyServiceMsgResponse,
  md5sum() { return '9cc62145209219f185101e72e3ae5c65'; },
  datatype() { return 'my_class_pkg/MyServiceMsg'; }
};
