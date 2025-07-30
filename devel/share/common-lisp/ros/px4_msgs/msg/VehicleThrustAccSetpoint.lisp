; Auto-generated. Do not edit!


(cl:in-package px4_msgs-msg)


;//! \htmlinclude VehicleThrustAccSetpoint.msg.html

(cl:defclass <VehicleThrustAccSetpoint> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:integer
    :initform 0)
   (thrust_acc_sp
    :reader thrust_acc_sp
    :initarg :thrust_acc_sp
    :type cl:float
    :initform 0.0)
   (rates_sp
    :reader rates_sp
    :initarg :rates_sp
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (model_ff
    :reader model_ff
    :initarg :model_ff
    :type cl:float
    :initform 0.0)
   (lead_information
    :reader lead_information
    :initarg :lead_information
    :type cl:float
    :initform 0.0))
)

(cl:defclass VehicleThrustAccSetpoint (<VehicleThrustAccSetpoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VehicleThrustAccSetpoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VehicleThrustAccSetpoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_msgs-msg:<VehicleThrustAccSetpoint> is deprecated: use px4_msgs-msg:VehicleThrustAccSetpoint instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <VehicleThrustAccSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_msgs-msg:timestamp-val is deprecated.  Use px4_msgs-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'thrust_acc_sp-val :lambda-list '(m))
(cl:defmethod thrust_acc_sp-val ((m <VehicleThrustAccSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_msgs-msg:thrust_acc_sp-val is deprecated.  Use px4_msgs-msg:thrust_acc_sp instead.")
  (thrust_acc_sp m))

(cl:ensure-generic-function 'rates_sp-val :lambda-list '(m))
(cl:defmethod rates_sp-val ((m <VehicleThrustAccSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_msgs-msg:rates_sp-val is deprecated.  Use px4_msgs-msg:rates_sp instead.")
  (rates_sp m))

(cl:ensure-generic-function 'model_ff-val :lambda-list '(m))
(cl:defmethod model_ff-val ((m <VehicleThrustAccSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_msgs-msg:model_ff-val is deprecated.  Use px4_msgs-msg:model_ff instead.")
  (model_ff m))

(cl:ensure-generic-function 'lead_information-val :lambda-list '(m))
(cl:defmethod lead_information-val ((m <VehicleThrustAccSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_msgs-msg:lead_information-val is deprecated.  Use px4_msgs-msg:lead_information instead.")
  (lead_information m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VehicleThrustAccSetpoint>) ostream)
  "Serializes a message object of type '<VehicleThrustAccSetpoint>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'timestamp)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thrust_acc_sp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'rates_sp))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'model_ff))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lead_information))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VehicleThrustAccSetpoint>) istream)
  "Deserializes a message object of type '<VehicleThrustAccSetpoint>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'timestamp)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thrust_acc_sp) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'rates_sp) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'rates_sp)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'model_ff) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lead_information) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VehicleThrustAccSetpoint>)))
  "Returns string type for a message object of type '<VehicleThrustAccSetpoint>"
  "px4_msgs/VehicleThrustAccSetpoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VehicleThrustAccSetpoint)))
  "Returns string type for a message object of type 'VehicleThrustAccSetpoint"
  "px4_msgs/VehicleThrustAccSetpoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VehicleThrustAccSetpoint>)))
  "Returns md5sum for a message object of type '<VehicleThrustAccSetpoint>"
  "5872a8a2a7714255ea85f043e820058d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VehicleThrustAccSetpoint)))
  "Returns md5sum for a message object of type 'VehicleThrustAccSetpoint"
  "5872a8a2a7714255ea85f043e820058d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VehicleThrustAccSetpoint>)))
  "Returns full string definition for message of type '<VehicleThrustAccSetpoint>"
  (cl:format cl:nil "~%uint64 timestamp        # time since system start (microseconds)~%~%~%~%float32 thrust_acc_sp          # thrust acceleration setpoint along the thrust axis, like amostly Z body axis.~%~%float32[3] rates_sp~%~%float32 model_ff~%float32 lead_information~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VehicleThrustAccSetpoint)))
  "Returns full string definition for message of type 'VehicleThrustAccSetpoint"
  (cl:format cl:nil "~%uint64 timestamp        # time since system start (microseconds)~%~%~%~%float32 thrust_acc_sp          # thrust acceleration setpoint along the thrust axis, like amostly Z body axis.~%~%float32[3] rates_sp~%~%float32 model_ff~%float32 lead_information~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VehicleThrustAccSetpoint>))
  (cl:+ 0
     8
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'rates_sp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VehicleThrustAccSetpoint>))
  "Converts a ROS message object to a list"
  (cl:list 'VehicleThrustAccSetpoint
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':thrust_acc_sp (thrust_acc_sp msg))
    (cl:cons ':rates_sp (rates_sp msg))
    (cl:cons ':model_ff (model_ff msg))
    (cl:cons ':lead_information (lead_information msg))
))
