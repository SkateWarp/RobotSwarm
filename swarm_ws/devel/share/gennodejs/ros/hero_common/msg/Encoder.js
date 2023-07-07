// Auto-generated. Do not edit!

// (in-package hero_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Encoder {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.left_ticks = null;
      this.right_ticks = null;
      this.left_diff = null;
      this.right_diff = null;
      this.left_dist = null;
      this.right_dist = null;
      this.timestep = null;
      this.left_speed = null;
      this.right_speed = null;
      this.left_speed_filtered = null;
      this.right_speed_filtered = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('left_ticks')) {
        this.left_ticks = initObj.left_ticks
      }
      else {
        this.left_ticks = 0.0;
      }
      if (initObj.hasOwnProperty('right_ticks')) {
        this.right_ticks = initObj.right_ticks
      }
      else {
        this.right_ticks = 0.0;
      }
      if (initObj.hasOwnProperty('left_diff')) {
        this.left_diff = initObj.left_diff
      }
      else {
        this.left_diff = 0.0;
      }
      if (initObj.hasOwnProperty('right_diff')) {
        this.right_diff = initObj.right_diff
      }
      else {
        this.right_diff = 0.0;
      }
      if (initObj.hasOwnProperty('left_dist')) {
        this.left_dist = initObj.left_dist
      }
      else {
        this.left_dist = 0.0;
      }
      if (initObj.hasOwnProperty('right_dist')) {
        this.right_dist = initObj.right_dist
      }
      else {
        this.right_dist = 0.0;
      }
      if (initObj.hasOwnProperty('timestep')) {
        this.timestep = initObj.timestep
      }
      else {
        this.timestep = 0.0;
      }
      if (initObj.hasOwnProperty('left_speed')) {
        this.left_speed = initObj.left_speed
      }
      else {
        this.left_speed = 0.0;
      }
      if (initObj.hasOwnProperty('right_speed')) {
        this.right_speed = initObj.right_speed
      }
      else {
        this.right_speed = 0.0;
      }
      if (initObj.hasOwnProperty('left_speed_filtered')) {
        this.left_speed_filtered = initObj.left_speed_filtered
      }
      else {
        this.left_speed_filtered = 0.0;
      }
      if (initObj.hasOwnProperty('right_speed_filtered')) {
        this.right_speed_filtered = initObj.right_speed_filtered
      }
      else {
        this.right_speed_filtered = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Encoder
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [left_ticks]
    bufferOffset = _serializer.float64(obj.left_ticks, buffer, bufferOffset);
    // Serialize message field [right_ticks]
    bufferOffset = _serializer.float64(obj.right_ticks, buffer, bufferOffset);
    // Serialize message field [left_diff]
    bufferOffset = _serializer.float64(obj.left_diff, buffer, bufferOffset);
    // Serialize message field [right_diff]
    bufferOffset = _serializer.float64(obj.right_diff, buffer, bufferOffset);
    // Serialize message field [left_dist]
    bufferOffset = _serializer.float64(obj.left_dist, buffer, bufferOffset);
    // Serialize message field [right_dist]
    bufferOffset = _serializer.float64(obj.right_dist, buffer, bufferOffset);
    // Serialize message field [timestep]
    bufferOffset = _serializer.float64(obj.timestep, buffer, bufferOffset);
    // Serialize message field [left_speed]
    bufferOffset = _serializer.float64(obj.left_speed, buffer, bufferOffset);
    // Serialize message field [right_speed]
    bufferOffset = _serializer.float64(obj.right_speed, buffer, bufferOffset);
    // Serialize message field [left_speed_filtered]
    bufferOffset = _serializer.float64(obj.left_speed_filtered, buffer, bufferOffset);
    // Serialize message field [right_speed_filtered]
    bufferOffset = _serializer.float64(obj.right_speed_filtered, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Encoder
    let len;
    let data = new Encoder(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [left_ticks]
    data.left_ticks = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_ticks]
    data.right_ticks = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_diff]
    data.left_diff = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_diff]
    data.right_diff = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_dist]
    data.left_dist = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_dist]
    data.right_dist = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [timestep]
    data.timestep = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_speed]
    data.left_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_speed]
    data.right_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_speed_filtered]
    data.left_speed_filtered = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_speed_filtered]
    data.right_speed_filtered = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 88;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hero_common/Encoder';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b5586cdbae9b740afc6a1f403a2d7dde';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float64 left_ticks
    float64 right_ticks
    float64 left_diff
    float64 right_diff
    float64 left_dist
    float64 right_dist
    float64 timestep
    float64 left_speed
    float64 right_speed
    float64 left_speed_filtered
    float64 right_speed_filtered
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Encoder(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.left_ticks !== undefined) {
      resolved.left_ticks = msg.left_ticks;
    }
    else {
      resolved.left_ticks = 0.0
    }

    if (msg.right_ticks !== undefined) {
      resolved.right_ticks = msg.right_ticks;
    }
    else {
      resolved.right_ticks = 0.0
    }

    if (msg.left_diff !== undefined) {
      resolved.left_diff = msg.left_diff;
    }
    else {
      resolved.left_diff = 0.0
    }

    if (msg.right_diff !== undefined) {
      resolved.right_diff = msg.right_diff;
    }
    else {
      resolved.right_diff = 0.0
    }

    if (msg.left_dist !== undefined) {
      resolved.left_dist = msg.left_dist;
    }
    else {
      resolved.left_dist = 0.0
    }

    if (msg.right_dist !== undefined) {
      resolved.right_dist = msg.right_dist;
    }
    else {
      resolved.right_dist = 0.0
    }

    if (msg.timestep !== undefined) {
      resolved.timestep = msg.timestep;
    }
    else {
      resolved.timestep = 0.0
    }

    if (msg.left_speed !== undefined) {
      resolved.left_speed = msg.left_speed;
    }
    else {
      resolved.left_speed = 0.0
    }

    if (msg.right_speed !== undefined) {
      resolved.right_speed = msg.right_speed;
    }
    else {
      resolved.right_speed = 0.0
    }

    if (msg.left_speed_filtered !== undefined) {
      resolved.left_speed_filtered = msg.left_speed_filtered;
    }
    else {
      resolved.left_speed_filtered = 0.0
    }

    if (msg.right_speed_filtered !== undefined) {
      resolved.right_speed_filtered = msg.right_speed_filtered;
    }
    else {
      resolved.right_speed_filtered = 0.0
    }

    return resolved;
    }
};

module.exports = Encoder;
