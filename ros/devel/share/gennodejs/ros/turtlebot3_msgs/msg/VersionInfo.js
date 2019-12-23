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

class VersionInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.hardware = null;
      this.firmware = null;
      this.software = null;
    }
    else {
      if (initObj.hasOwnProperty('hardware')) {
        this.hardware = initObj.hardware
      }
      else {
        this.hardware = '';
      }
      if (initObj.hasOwnProperty('firmware')) {
        this.firmware = initObj.firmware
      }
      else {
        this.firmware = '';
      }
      if (initObj.hasOwnProperty('software')) {
        this.software = initObj.software
      }
      else {
        this.software = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VersionInfo
    // Serialize message field [hardware]
    bufferOffset = _serializer.string(obj.hardware, buffer, bufferOffset);
    // Serialize message field [firmware]
    bufferOffset = _serializer.string(obj.firmware, buffer, bufferOffset);
    // Serialize message field [software]
    bufferOffset = _serializer.string(obj.software, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VersionInfo
    let len;
    let data = new VersionInfo(null);
    // Deserialize message field [hardware]
    data.hardware = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [firmware]
    data.firmware = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [software]
    data.software = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.hardware.length;
    length += object.firmware.length;
    length += object.software.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'turtlebot3_msgs/VersionInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '43e0361461af2970a33107409403ef3c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ########################################
    # Messages
    ########################################
    string hardware   # <yyyy>.<mm>.<dd>        : hardware version of Turtlebot3 (ex. 2017.05.23)
    string firmware   # <major>.<minor>.<patch> : firmware version of OpenCR
    string software   # <major>.<minor>.<patch> : software version of Turtlebot3 ROS packages
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VersionInfo(null);
    if (msg.hardware !== undefined) {
      resolved.hardware = msg.hardware;
    }
    else {
      resolved.hardware = ''
    }

    if (msg.firmware !== undefined) {
      resolved.firmware = msg.firmware;
    }
    else {
      resolved.firmware = ''
    }

    if (msg.software !== undefined) {
      resolved.software = msg.software;
    }
    else {
      resolved.software = ''
    }

    return resolved;
    }
};

module.exports = VersionInfo;
