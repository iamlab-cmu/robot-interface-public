// Auto-generated. Do not edit!

// (in-package franka_action_lib.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let RobolibStatus = require('../msg/RobolibStatus.js');

//-----------------------------------------------------------

class GetCurrentRobolibStatusCmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCurrentRobolibStatusCmdRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentRobolibStatusCmdRequest
    let len;
    let data = new GetCurrentRobolibStatusCmdRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'franka_action_lib/GetCurrentRobolibStatusCmdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentRobolibStatusCmdRequest(null);
    return resolved;
    }
};

class GetCurrentRobolibStatusCmdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robolib_status = null;
    }
    else {
      if (initObj.hasOwnProperty('robolib_status')) {
        this.robolib_status = initObj.robolib_status
      }
      else {
        this.robolib_status = new RobolibStatus();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCurrentRobolibStatusCmdResponse
    // Serialize message field [robolib_status]
    bufferOffset = RobolibStatus.serialize(obj.robolib_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentRobolibStatusCmdResponse
    let len;
    let data = new GetCurrentRobolibStatusCmdResponse(null);
    // Deserialize message field [robolib_status]
    data.robolib_status = RobolibStatus.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += RobolibStatus.getMessageSize(object.robolib_status);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'franka_action_lib/GetCurrentRobolibStatusCmdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4a8606837e9f22dc33518a29b863eeb4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    RobolibStatus robolib_status
    
    ================================================================================
    MSG: franka_action_lib/RobolibStatus
    # Franka robot state
    std_msgs/Header header
    bool is_ready
    string error_description
    bool is_fresh
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentRobolibStatusCmdResponse(null);
    if (msg.robolib_status !== undefined) {
      resolved.robolib_status = RobolibStatus.Resolve(msg.robolib_status)
    }
    else {
      resolved.robolib_status = new RobolibStatus()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetCurrentRobolibStatusCmdRequest,
  Response: GetCurrentRobolibStatusCmdResponse,
  md5sum() { return '4a8606837e9f22dc33518a29b863eeb4'; },
  datatype() { return 'franka_action_lib/GetCurrentRobolibStatusCmd'; }
};
