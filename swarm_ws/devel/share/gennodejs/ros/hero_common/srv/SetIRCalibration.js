// Auto-generated. Do not edit!

// (in-package hero_common.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetIRCalibrationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.IR0 = null;
      this.IR1 = null;
      this.IR2 = null;
      this.IR3 = null;
      this.IR4 = null;
      this.IR5 = null;
      this.IR6 = null;
      this.IR7 = null;
    }
    else {
      if (initObj.hasOwnProperty('IR0')) {
        this.IR0 = initObj.IR0
      }
      else {
        this.IR0 = 0.0;
      }
      if (initObj.hasOwnProperty('IR1')) {
        this.IR1 = initObj.IR1
      }
      else {
        this.IR1 = 0.0;
      }
      if (initObj.hasOwnProperty('IR2')) {
        this.IR2 = initObj.IR2
      }
      else {
        this.IR2 = 0.0;
      }
      if (initObj.hasOwnProperty('IR3')) {
        this.IR3 = initObj.IR3
      }
      else {
        this.IR3 = 0.0;
      }
      if (initObj.hasOwnProperty('IR4')) {
        this.IR4 = initObj.IR4
      }
      else {
        this.IR4 = 0.0;
      }
      if (initObj.hasOwnProperty('IR5')) {
        this.IR5 = initObj.IR5
      }
      else {
        this.IR5 = 0.0;
      }
      if (initObj.hasOwnProperty('IR6')) {
        this.IR6 = initObj.IR6
      }
      else {
        this.IR6 = 0.0;
      }
      if (initObj.hasOwnProperty('IR7')) {
        this.IR7 = initObj.IR7
      }
      else {
        this.IR7 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetIRCalibrationRequest
    // Serialize message field [IR0]
    bufferOffset = _serializer.float64(obj.IR0, buffer, bufferOffset);
    // Serialize message field [IR1]
    bufferOffset = _serializer.float64(obj.IR1, buffer, bufferOffset);
    // Serialize message field [IR2]
    bufferOffset = _serializer.float64(obj.IR2, buffer, bufferOffset);
    // Serialize message field [IR3]
    bufferOffset = _serializer.float64(obj.IR3, buffer, bufferOffset);
    // Serialize message field [IR4]
    bufferOffset = _serializer.float64(obj.IR4, buffer, bufferOffset);
    // Serialize message field [IR5]
    bufferOffset = _serializer.float64(obj.IR5, buffer, bufferOffset);
    // Serialize message field [IR6]
    bufferOffset = _serializer.float64(obj.IR6, buffer, bufferOffset);
    // Serialize message field [IR7]
    bufferOffset = _serializer.float64(obj.IR7, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIRCalibrationRequest
    let len;
    let data = new SetIRCalibrationRequest(null);
    // Deserialize message field [IR0]
    data.IR0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [IR1]
    data.IR1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [IR2]
    data.IR2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [IR3]
    data.IR3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [IR4]
    data.IR4 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [IR5]
    data.IR5 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [IR6]
    data.IR6 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [IR7]
    data.IR7 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hero_common/SetIRCalibrationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '27748d949ad62e65750ad22074426005';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 IR0 # alpha value for IR0
    float64 IR1 # alpha value for IR1
    float64 IR2 # alpha value for IR2
    float64 IR3 # alpha value for IR3
    float64 IR4 # alpha value for IR4
    float64 IR5 # alpha value for IR5
    float64 IR6 # alpha value for IR6
    float64 IR7 # alpha value for IR7
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetIRCalibrationRequest(null);
    if (msg.IR0 !== undefined) {
      resolved.IR0 = msg.IR0;
    }
    else {
      resolved.IR0 = 0.0
    }

    if (msg.IR1 !== undefined) {
      resolved.IR1 = msg.IR1;
    }
    else {
      resolved.IR1 = 0.0
    }

    if (msg.IR2 !== undefined) {
      resolved.IR2 = msg.IR2;
    }
    else {
      resolved.IR2 = 0.0
    }

    if (msg.IR3 !== undefined) {
      resolved.IR3 = msg.IR3;
    }
    else {
      resolved.IR3 = 0.0
    }

    if (msg.IR4 !== undefined) {
      resolved.IR4 = msg.IR4;
    }
    else {
      resolved.IR4 = 0.0
    }

    if (msg.IR5 !== undefined) {
      resolved.IR5 = msg.IR5;
    }
    else {
      resolved.IR5 = 0.0
    }

    if (msg.IR6 !== undefined) {
      resolved.IR6 = msg.IR6;
    }
    else {
      resolved.IR6 = 0.0
    }

    if (msg.IR7 !== undefined) {
      resolved.IR7 = msg.IR7;
    }
    else {
      resolved.IR7 = 0.0
    }

    return resolved;
    }
};

class SetIRCalibrationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetIRCalibrationResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIRCalibrationResponse
    let len;
    let data = new SetIRCalibrationResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hero_common/SetIRCalibrationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success   # indicate successful run of triggered service
    string message # informational, e.g. for error messages
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetIRCalibrationResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SetIRCalibrationRequest,
  Response: SetIRCalibrationResponse,
  md5sum() { return 'fc55fd34ad2ef676815ba2e14521ac73'; },
  datatype() { return 'hero_common/SetIRCalibration'; }
};
