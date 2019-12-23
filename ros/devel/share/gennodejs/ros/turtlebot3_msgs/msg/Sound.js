// Auto-generated. Do not edit!

// (in-package turtlebot3_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Sound {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Sound
    // Serialize message field [value]
    bufferOffset = _serializer.uint8(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Sound
    let len;
    let data = new Sound(null);
    // Deserialize message field [value]
    data.value = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'turtlebot3_msgs/Sound';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e1f8c7f8a9a61383b5734fbdeca2f99a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ########################################
    # CONSTANTS
    ########################################
    uint8 OFF           = 0
    uint8 ON            = 1
    uint8 LOW_BATTERY   = 2
    uint8 ERROR         = 3
    uint8 BUTTON1       = 4
    uint8 BUTTON2       = 5
    
    ########################################
    # Messages
    ########################################
    uint8 value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Sound(null);
    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
    }

    return resolved;
    }
};

// Constants for message
Sound.Constants = {
  OFF: 0,
  ON: 1,
  LOW_BATTERY: 2,
  ERROR: 3,
  BUTTON1: 4,
  BUTTON2: 5,
}

module.exports = Sound;
