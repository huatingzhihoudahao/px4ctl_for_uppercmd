// Auto-generated. Do not edit!

// (in-package px4_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class VehicleThrustAccSetpoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.thrust_acc_sp = null;
      this.rates_sp = null;
      this.model_ff = null;
      this.lead_information = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0;
      }
      if (initObj.hasOwnProperty('thrust_acc_sp')) {
        this.thrust_acc_sp = initObj.thrust_acc_sp
      }
      else {
        this.thrust_acc_sp = 0.0;
      }
      if (initObj.hasOwnProperty('rates_sp')) {
        this.rates_sp = initObj.rates_sp
      }
      else {
        this.rates_sp = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('model_ff')) {
        this.model_ff = initObj.model_ff
      }
      else {
        this.model_ff = 0.0;
      }
      if (initObj.hasOwnProperty('lead_information')) {
        this.lead_information = initObj.lead_information
      }
      else {
        this.lead_information = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VehicleThrustAccSetpoint
    // Serialize message field [timestamp]
    bufferOffset = _serializer.uint64(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [thrust_acc_sp]
    bufferOffset = _serializer.float32(obj.thrust_acc_sp, buffer, bufferOffset);
    // Check that the constant length array field [rates_sp] has the right length
    if (obj.rates_sp.length !== 3) {
      throw new Error('Unable to serialize array field rates_sp - length must be 3')
    }
    // Serialize message field [rates_sp]
    bufferOffset = _arraySerializer.float32(obj.rates_sp, buffer, bufferOffset, 3);
    // Serialize message field [model_ff]
    bufferOffset = _serializer.float32(obj.model_ff, buffer, bufferOffset);
    // Serialize message field [lead_information]
    bufferOffset = _serializer.float32(obj.lead_information, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VehicleThrustAccSetpoint
    let len;
    let data = new VehicleThrustAccSetpoint(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [thrust_acc_sp]
    data.thrust_acc_sp = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rates_sp]
    data.rates_sp = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [model_ff]
    data.model_ff = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lead_information]
    data.lead_information = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px4_msgs/VehicleThrustAccSetpoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5872a8a2a7714255ea85f043e820058d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint64 timestamp        # time since system start (microseconds)
    
    
    
    float32 thrust_acc_sp          # thrust acceleration setpoint along the thrust axis, like amostly Z body axis.
    
    float32[3] rates_sp
    
    float32 model_ff
    float32 lead_information
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VehicleThrustAccSetpoint(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0
    }

    if (msg.thrust_acc_sp !== undefined) {
      resolved.thrust_acc_sp = msg.thrust_acc_sp;
    }
    else {
      resolved.thrust_acc_sp = 0.0
    }

    if (msg.rates_sp !== undefined) {
      resolved.rates_sp = msg.rates_sp;
    }
    else {
      resolved.rates_sp = new Array(3).fill(0)
    }

    if (msg.model_ff !== undefined) {
      resolved.model_ff = msg.model_ff;
    }
    else {
      resolved.model_ff = 0.0
    }

    if (msg.lead_information !== undefined) {
      resolved.lead_information = msg.lead_information;
    }
    else {
      resolved.lead_information = 0.0
    }

    return resolved;
    }
};

module.exports = VehicleThrustAccSetpoint;
