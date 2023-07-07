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

class SetMotorRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_motor_pwm = null;
      this.right_motor_pwm = null;
    }
    else {
      if (initObj.hasOwnProperty('left_motor_pwm')) {
        this.left_motor_pwm = initObj.left_motor_pwm
      }
      else {
        this.left_motor_pwm = 0;
      }
      if (initObj.hasOwnProperty('right_motor_pwm')) {
        this.right_motor_pwm = initObj.right_motor_pwm
      }
      else {
        this.right_motor_pwm = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetMotorRequest
    // Serialize message field [left_motor_pwm]
    bufferOffset = _serializer.int16(obj.left_motor_pwm, buffer, bufferOffset);
    // Serialize message field [right_motor_pwm]
    bufferOffset = _serializer.int16(obj.right_motor_pwm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMotorRequest
    let len;
    let data = new SetMotorRequest(null);
    // Deserialize message field [left_motor_pwm]
    data.left_motor_pwm = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [right_motor_pwm]
    data.right_motor_pwm = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hero_common/SetMotorRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '44d4249d207ce6a89de3d72080e4d0b5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 left_motor_pwm # motor deadzone pwm
    int16 right_motor_pwm # motor deadzone pwm
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetMotorRequest(null);
    if (msg.left_motor_pwm !== undefined) {
      resolved.left_motor_pwm = msg.left_motor_pwm;
    }
    else {
      resolved.left_motor_pwm = 0
    }

    if (msg.right_motor_pwm !== undefined) {
      resolved.right_motor_pwm = msg.right_motor_pwm;
    }
    else {
      resolved.right_motor_pwm = 0
    }

    return resolved;
    }
};

class SetMotorResponse {
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
    // Serializes a message object of type SetMotorResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMotorResponse
    let len;
    let data = new SetMotorResponse(null);
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
    return 'hero_common/SetMotorResponse';
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
    const resolved = new SetMotorResponse(null);
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
  Request: SetMotorRequest,
  Response: SetMotorResponse,
  md5sum() { return '07e32bced0fc14456815cdd097819b89'; },
  datatype() { return 'hero_common/SetMotor'; }
};
