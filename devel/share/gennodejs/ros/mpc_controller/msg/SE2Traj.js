// Auto-generated. Do not edit!

// (in-package mpc_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class SE2Traj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_time = null;
      this.pos_pts = null;
      this.angle_pts = null;
      this.init_v = null;
      this.init_a = null;
      this.posT_pts = null;
      this.angleT_pts = null;
    }
    else {
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('pos_pts')) {
        this.pos_pts = initObj.pos_pts
      }
      else {
        this.pos_pts = [];
      }
      if (initObj.hasOwnProperty('angle_pts')) {
        this.angle_pts = initObj.angle_pts
      }
      else {
        this.angle_pts = [];
      }
      if (initObj.hasOwnProperty('init_v')) {
        this.init_v = initObj.init_v
      }
      else {
        this.init_v = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('init_a')) {
        this.init_a = initObj.init_a
      }
      else {
        this.init_a = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('posT_pts')) {
        this.posT_pts = initObj.posT_pts
      }
      else {
        this.posT_pts = [];
      }
      if (initObj.hasOwnProperty('angleT_pts')) {
        this.angleT_pts = initObj.angleT_pts
      }
      else {
        this.angleT_pts = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SE2Traj
    // Serialize message field [start_time]
    bufferOffset = _serializer.time(obj.start_time, buffer, bufferOffset);
    // Serialize message field [pos_pts]
    // Serialize the length for message field [pos_pts]
    bufferOffset = _serializer.uint32(obj.pos_pts.length, buffer, bufferOffset);
    obj.pos_pts.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [angle_pts]
    // Serialize the length for message field [angle_pts]
    bufferOffset = _serializer.uint32(obj.angle_pts.length, buffer, bufferOffset);
    obj.angle_pts.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [init_v]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.init_v, buffer, bufferOffset);
    // Serialize message field [init_a]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.init_a, buffer, bufferOffset);
    // Serialize message field [posT_pts]
    bufferOffset = _arraySerializer.float64(obj.posT_pts, buffer, bufferOffset, null);
    // Serialize message field [angleT_pts]
    bufferOffset = _arraySerializer.float64(obj.angleT_pts, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SE2Traj
    let len;
    let data = new SE2Traj(null);
    // Deserialize message field [start_time]
    data.start_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [pos_pts]
    // Deserialize array length for message field [pos_pts]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pos_pts = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pos_pts[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [angle_pts]
    // Deserialize array length for message field [angle_pts]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.angle_pts = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.angle_pts[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [init_v]
    data.init_v = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [init_a]
    data.init_a = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [posT_pts]
    data.posT_pts = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [angleT_pts]
    data.angleT_pts = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.pos_pts.length;
    length += 24 * object.angle_pts.length;
    length += 8 * object.posT_pts.length;
    length += 8 * object.angleT_pts.length;
    return length + 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mpc_controller/SE2Traj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f921a322e80c694816eb9412daf79b62';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MINCO trajectory
    
    time start_time
    geometry_msgs/Point[] pos_pts
    geometry_msgs/Point[] angle_pts
    geometry_msgs/Vector3 init_v
    geometry_msgs/Vector3 init_a
    float64[] posT_pts
    float64[] angleT_pts
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SE2Traj(null);
    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = {secs: 0, nsecs: 0}
    }

    if (msg.pos_pts !== undefined) {
      resolved.pos_pts = new Array(msg.pos_pts.length);
      for (let i = 0; i < resolved.pos_pts.length; ++i) {
        resolved.pos_pts[i] = geometry_msgs.msg.Point.Resolve(msg.pos_pts[i]);
      }
    }
    else {
      resolved.pos_pts = []
    }

    if (msg.angle_pts !== undefined) {
      resolved.angle_pts = new Array(msg.angle_pts.length);
      for (let i = 0; i < resolved.angle_pts.length; ++i) {
        resolved.angle_pts[i] = geometry_msgs.msg.Point.Resolve(msg.angle_pts[i]);
      }
    }
    else {
      resolved.angle_pts = []
    }

    if (msg.init_v !== undefined) {
      resolved.init_v = geometry_msgs.msg.Vector3.Resolve(msg.init_v)
    }
    else {
      resolved.init_v = new geometry_msgs.msg.Vector3()
    }

    if (msg.init_a !== undefined) {
      resolved.init_a = geometry_msgs.msg.Vector3.Resolve(msg.init_a)
    }
    else {
      resolved.init_a = new geometry_msgs.msg.Vector3()
    }

    if (msg.posT_pts !== undefined) {
      resolved.posT_pts = msg.posT_pts;
    }
    else {
      resolved.posT_pts = []
    }

    if (msg.angleT_pts !== undefined) {
      resolved.angleT_pts = msg.angleT_pts;
    }
    else {
      resolved.angleT_pts = []
    }

    return resolved;
    }
};

module.exports = SE2Traj;
