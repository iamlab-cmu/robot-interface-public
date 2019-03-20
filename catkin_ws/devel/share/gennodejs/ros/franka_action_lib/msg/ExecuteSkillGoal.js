// Auto-generated. Do not edit!

// (in-package franka_action_lib.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ExecuteSkillGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.skill_type = null;
      this.skill_description = null;
      this.meta_skill_type = null;
      this.meta_skill_id = null;
      this.sensor_topics = null;
      this.sensor_value_sizes = null;
      this.initial_sensor_values = null;
      this.traj_gen_type = null;
      this.num_traj_gen_params = null;
      this.traj_gen_params = null;
      this.feedback_controller_type = null;
      this.num_feedback_controller_params = null;
      this.feedback_controller_params = null;
      this.termination_type = null;
      this.num_termination_params = null;
      this.termination_params = null;
      this.timer_type = null;
      this.num_timer_params = null;
      this.timer_params = null;
    }
    else {
      if (initObj.hasOwnProperty('skill_type')) {
        this.skill_type = initObj.skill_type
      }
      else {
        this.skill_type = 0;
      }
      if (initObj.hasOwnProperty('skill_description')) {
        this.skill_description = initObj.skill_description
      }
      else {
        this.skill_description = '';
      }
      if (initObj.hasOwnProperty('meta_skill_type')) {
        this.meta_skill_type = initObj.meta_skill_type
      }
      else {
        this.meta_skill_type = 0;
      }
      if (initObj.hasOwnProperty('meta_skill_id')) {
        this.meta_skill_id = initObj.meta_skill_id
      }
      else {
        this.meta_skill_id = 0;
      }
      if (initObj.hasOwnProperty('sensor_topics')) {
        this.sensor_topics = initObj.sensor_topics
      }
      else {
        this.sensor_topics = [];
      }
      if (initObj.hasOwnProperty('sensor_value_sizes')) {
        this.sensor_value_sizes = initObj.sensor_value_sizes
      }
      else {
        this.sensor_value_sizes = [];
      }
      if (initObj.hasOwnProperty('initial_sensor_values')) {
        this.initial_sensor_values = initObj.initial_sensor_values
      }
      else {
        this.initial_sensor_values = [];
      }
      if (initObj.hasOwnProperty('traj_gen_type')) {
        this.traj_gen_type = initObj.traj_gen_type
      }
      else {
        this.traj_gen_type = 0;
      }
      if (initObj.hasOwnProperty('num_traj_gen_params')) {
        this.num_traj_gen_params = initObj.num_traj_gen_params
      }
      else {
        this.num_traj_gen_params = 0;
      }
      if (initObj.hasOwnProperty('traj_gen_params')) {
        this.traj_gen_params = initObj.traj_gen_params
      }
      else {
        this.traj_gen_params = [];
      }
      if (initObj.hasOwnProperty('feedback_controller_type')) {
        this.feedback_controller_type = initObj.feedback_controller_type
      }
      else {
        this.feedback_controller_type = 0;
      }
      if (initObj.hasOwnProperty('num_feedback_controller_params')) {
        this.num_feedback_controller_params = initObj.num_feedback_controller_params
      }
      else {
        this.num_feedback_controller_params = 0;
      }
      if (initObj.hasOwnProperty('feedback_controller_params')) {
        this.feedback_controller_params = initObj.feedback_controller_params
      }
      else {
        this.feedback_controller_params = [];
      }
      if (initObj.hasOwnProperty('termination_type')) {
        this.termination_type = initObj.termination_type
      }
      else {
        this.termination_type = 0;
      }
      if (initObj.hasOwnProperty('num_termination_params')) {
        this.num_termination_params = initObj.num_termination_params
      }
      else {
        this.num_termination_params = 0;
      }
      if (initObj.hasOwnProperty('termination_params')) {
        this.termination_params = initObj.termination_params
      }
      else {
        this.termination_params = [];
      }
      if (initObj.hasOwnProperty('timer_type')) {
        this.timer_type = initObj.timer_type
      }
      else {
        this.timer_type = 0;
      }
      if (initObj.hasOwnProperty('num_timer_params')) {
        this.num_timer_params = initObj.num_timer_params
      }
      else {
        this.num_timer_params = 0;
      }
      if (initObj.hasOwnProperty('timer_params')) {
        this.timer_params = initObj.timer_params
      }
      else {
        this.timer_params = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExecuteSkillGoal
    // Serialize message field [skill_type]
    bufferOffset = _serializer.int64(obj.skill_type, buffer, bufferOffset);
    // Serialize message field [skill_description]
    bufferOffset = _serializer.string(obj.skill_description, buffer, bufferOffset);
    // Serialize message field [meta_skill_type]
    bufferOffset = _serializer.int64(obj.meta_skill_type, buffer, bufferOffset);
    // Serialize message field [meta_skill_id]
    bufferOffset = _serializer.int64(obj.meta_skill_id, buffer, bufferOffset);
    // Serialize message field [sensor_topics]
    bufferOffset = _arraySerializer.string(obj.sensor_topics, buffer, bufferOffset, null);
    // Serialize message field [sensor_value_sizes]
    bufferOffset = _arraySerializer.uint64(obj.sensor_value_sizes, buffer, bufferOffset, null);
    // Serialize message field [initial_sensor_values]
    bufferOffset = _arraySerializer.float64(obj.initial_sensor_values, buffer, bufferOffset, null);
    // Serialize message field [traj_gen_type]
    bufferOffset = _serializer.int64(obj.traj_gen_type, buffer, bufferOffset);
    // Serialize message field [num_traj_gen_params]
    bufferOffset = _serializer.uint64(obj.num_traj_gen_params, buffer, bufferOffset);
    // Serialize message field [traj_gen_params]
    bufferOffset = _arraySerializer.float64(obj.traj_gen_params, buffer, bufferOffset, null);
    // Serialize message field [feedback_controller_type]
    bufferOffset = _serializer.int64(obj.feedback_controller_type, buffer, bufferOffset);
    // Serialize message field [num_feedback_controller_params]
    bufferOffset = _serializer.uint64(obj.num_feedback_controller_params, buffer, bufferOffset);
    // Serialize message field [feedback_controller_params]
    bufferOffset = _arraySerializer.float64(obj.feedback_controller_params, buffer, bufferOffset, null);
    // Serialize message field [termination_type]
    bufferOffset = _serializer.int64(obj.termination_type, buffer, bufferOffset);
    // Serialize message field [num_termination_params]
    bufferOffset = _serializer.uint64(obj.num_termination_params, buffer, bufferOffset);
    // Serialize message field [termination_params]
    bufferOffset = _arraySerializer.float64(obj.termination_params, buffer, bufferOffset, null);
    // Serialize message field [timer_type]
    bufferOffset = _serializer.int64(obj.timer_type, buffer, bufferOffset);
    // Serialize message field [num_timer_params]
    bufferOffset = _serializer.uint64(obj.num_timer_params, buffer, bufferOffset);
    // Serialize message field [timer_params]
    bufferOffset = _arraySerializer.float64(obj.timer_params, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExecuteSkillGoal
    let len;
    let data = new ExecuteSkillGoal(null);
    // Deserialize message field [skill_type]
    data.skill_type = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [skill_description]
    data.skill_description = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [meta_skill_type]
    data.meta_skill_type = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [meta_skill_id]
    data.meta_skill_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [sensor_topics]
    data.sensor_topics = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [sensor_value_sizes]
    data.sensor_value_sizes = _arrayDeserializer.uint64(buffer, bufferOffset, null)
    // Deserialize message field [initial_sensor_values]
    data.initial_sensor_values = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [traj_gen_type]
    data.traj_gen_type = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [num_traj_gen_params]
    data.num_traj_gen_params = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [traj_gen_params]
    data.traj_gen_params = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [feedback_controller_type]
    data.feedback_controller_type = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [num_feedback_controller_params]
    data.num_feedback_controller_params = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [feedback_controller_params]
    data.feedback_controller_params = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [termination_type]
    data.termination_type = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [num_termination_params]
    data.num_termination_params = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [termination_params]
    data.termination_params = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [timer_type]
    data.timer_type = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [num_timer_params]
    data.num_timer_params = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [timer_params]
    data.timer_params = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.skill_description.length;
    object.sensor_topics.forEach((val) => {
      length += 4 + val.length;
    });
    length += 8 * object.sensor_value_sizes.length;
    length += 8 * object.initial_sensor_values.length;
    length += 8 * object.traj_gen_params.length;
    length += 8 * object.feedback_controller_params.length;
    length += 8 * object.termination_params.length;
    length += 8 * object.timer_params.length;
    return length + 120;
  }

  static datatype() {
    // Returns string type for a message object
    return 'franka_action_lib/ExecuteSkillGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0a5a9702eeb075de3d4f7812c5b9bf0c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    int64 skill_type
    string skill_description
    int64 meta_skill_type
    int64 meta_skill_id
    
    # Sensor topic to subscribe to
    string[] sensor_topics
    uint64[] sensor_value_sizes
    float64[] initial_sensor_values
    
    # traj gen
    int64 traj_gen_type
    uint64 num_traj_gen_params
    float64[] traj_gen_params
    
    # fbc
    int64 feedback_controller_type
    uint64 num_feedback_controller_params
    float64[] feedback_controller_params
    
    # termination
    int64 termination_type
    uint64 num_termination_params
    float64[] termination_params 
    
    # timer
    int64 timer_type
    uint64 num_timer_params
    float64[] timer_params
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExecuteSkillGoal(null);
    if (msg.skill_type !== undefined) {
      resolved.skill_type = msg.skill_type;
    }
    else {
      resolved.skill_type = 0
    }

    if (msg.skill_description !== undefined) {
      resolved.skill_description = msg.skill_description;
    }
    else {
      resolved.skill_description = ''
    }

    if (msg.meta_skill_type !== undefined) {
      resolved.meta_skill_type = msg.meta_skill_type;
    }
    else {
      resolved.meta_skill_type = 0
    }

    if (msg.meta_skill_id !== undefined) {
      resolved.meta_skill_id = msg.meta_skill_id;
    }
    else {
      resolved.meta_skill_id = 0
    }

    if (msg.sensor_topics !== undefined) {
      resolved.sensor_topics = msg.sensor_topics;
    }
    else {
      resolved.sensor_topics = []
    }

    if (msg.sensor_value_sizes !== undefined) {
      resolved.sensor_value_sizes = msg.sensor_value_sizes;
    }
    else {
      resolved.sensor_value_sizes = []
    }

    if (msg.initial_sensor_values !== undefined) {
      resolved.initial_sensor_values = msg.initial_sensor_values;
    }
    else {
      resolved.initial_sensor_values = []
    }

    if (msg.traj_gen_type !== undefined) {
      resolved.traj_gen_type = msg.traj_gen_type;
    }
    else {
      resolved.traj_gen_type = 0
    }

    if (msg.num_traj_gen_params !== undefined) {
      resolved.num_traj_gen_params = msg.num_traj_gen_params;
    }
    else {
      resolved.num_traj_gen_params = 0
    }

    if (msg.traj_gen_params !== undefined) {
      resolved.traj_gen_params = msg.traj_gen_params;
    }
    else {
      resolved.traj_gen_params = []
    }

    if (msg.feedback_controller_type !== undefined) {
      resolved.feedback_controller_type = msg.feedback_controller_type;
    }
    else {
      resolved.feedback_controller_type = 0
    }

    if (msg.num_feedback_controller_params !== undefined) {
      resolved.num_feedback_controller_params = msg.num_feedback_controller_params;
    }
    else {
      resolved.num_feedback_controller_params = 0
    }

    if (msg.feedback_controller_params !== undefined) {
      resolved.feedback_controller_params = msg.feedback_controller_params;
    }
    else {
      resolved.feedback_controller_params = []
    }

    if (msg.termination_type !== undefined) {
      resolved.termination_type = msg.termination_type;
    }
    else {
      resolved.termination_type = 0
    }

    if (msg.num_termination_params !== undefined) {
      resolved.num_termination_params = msg.num_termination_params;
    }
    else {
      resolved.num_termination_params = 0
    }

    if (msg.termination_params !== undefined) {
      resolved.termination_params = msg.termination_params;
    }
    else {
      resolved.termination_params = []
    }

    if (msg.timer_type !== undefined) {
      resolved.timer_type = msg.timer_type;
    }
    else {
      resolved.timer_type = 0
    }

    if (msg.num_timer_params !== undefined) {
      resolved.num_timer_params = msg.num_timer_params;
    }
    else {
      resolved.num_timer_params = 0
    }

    if (msg.timer_params !== undefined) {
      resolved.timer_params = msg.timer_params;
    }
    else {
      resolved.timer_params = []
    }

    return resolved;
    }
};

module.exports = ExecuteSkillGoal;
