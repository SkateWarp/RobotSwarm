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

class SetPIDRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lkp = null;
      this.lki = null;
      this.lkd = null;
      this.rkp = null;
      this.rki = null;
      this.rkd = null;
    }
    else {
      if (initObj.hasOwnProperty('lkp')) {
        this.lkp = initObj.lkp
      }
      else {
        this.lkp = 0.0;
      }
      if (initObj.hasOwnProperty('lki')) {
        this.lki = initObj.lki
      }
      else {
        this.lki = 0.0;
      }
      if (initObj.hasOwnProperty('lkd')) {
        this.lkd = initObj.lkd
      }
      else {
        this.lkd = 0.0;
      }
      if (initObj.hasOwnProperty('rkp')) {
        this.rkp = initObj.rkp
      }
      else {
        this.rkp = 0.0;
      }
      if (initObj.hasOwnProperty('rki')) {
        this.rki = initObj.rki
      }
      else {
        this.rki = 0.0;
      }
      if (initObj.hasOwnProperty('rkd')) {
        this.rkd = initObj.rkd
      }
      else {
        this.rkd = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetPIDRequest
    // Serialize message field [lkp]
    bufferOffset = _serializer.float32(obj.lkp, buffer, bufferOffset);
    // Serialize message field [lki]
    bufferOffset = _serializer.float32(obj.lki, buffer, bufferOffset);
    // Serialize message field [lkd]
    bufferOffset = _serializer.float32(obj.lkd, buffer, bufferOffset);
    // Serialize message field [rkp]
    bufferOffset = _serializer.float32(obj.rkp, buffer, bufferOffset);
    // Serialize message field [rki]
    bufferOffset = _serializer.float32(obj.rki, buffer, bufferOffset);
    // Serialize message field [rkd]
    bufferOffset = _serializer.float32(obj.rkd, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetPIDRequest
    let len;
    let data = new SetPIDRequest(null);
    // Deserialize message field [lkp]
    data.lkp = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lki]
    data.lki = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lkd]
    data.lkd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rkp]
    data.rkp = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rki]
    data.rki = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rkd]
    data.rkd = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hero_common/SetPIDRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f0d0bdba02bc62a48d52638e38dc8a4b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 lkp # proportional term left motor
    float32 lki # intergrative term left motor
    float32 lkd # derivative term left motor
    float32 rkp # proportional term right motor
    float32 rki # intergrative term right motor
    float32 rkd # derivative term right motor
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetPIDRequest(null);
    if (msg.lkp !== undefined) {
      resolved.lkp = msg.lkp;
    }
    else {
      resolved.lkp = 0.0
    }

    if (msg.lki !== undefined) {
      resolved.lki = msg.lki;
    }
    else {
      resolved.lki = 0.0
    }

    if (msg.lkd !== undefined) {
      resolved.lkd = msg.lkd;
    }
    else {
      resolved.lkd = 0.0
    }

    if (msg.rkp !== undefined) {
      resolved.rkp = msg.rkp;
    }
    else {
      resolved.rkp = 0.0
    }

    if (msg.rki !== undefined) {
      resolved.rki = msg.rki;
    }
    else {
      resolved.rki = 0.0
    }

    if (msg.rkd !== undefined) {
      resolved.rkd = msg.rkd;
    }
    else {
      resolved.rkd = 0.0
    }

    return resolved;
    }
};

class SetPIDResponse {
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
    // Serializes a message object of type SetPIDResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetPIDResponse
    let len;
    let data = new SetPIDResponse(null);
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
    return 'hero_common/SetPIDResponse';
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
    const resolved = new SetPIDResponse(null);
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
  Request: SetPIDRequest,
  Response: SetPIDResponse,
  md5sum() { return '69c16eccc66fd6b477afb5e5500b4b47'; },
  datatype() { return 'hero_common/SetPID'; }
};
