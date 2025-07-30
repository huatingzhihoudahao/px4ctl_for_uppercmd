;; Auto-generated. Do not edit!


(when (boundp 'px4_msgs::VehicleThrustAccSetpoint)
  (if (not (find-package "PX4_MSGS"))
    (make-package "PX4_MSGS"))
  (shadow 'VehicleThrustAccSetpoint (find-package "PX4_MSGS")))
(unless (find-package "PX4_MSGS::VEHICLETHRUSTACCSETPOINT")
  (make-package "PX4_MSGS::VEHICLETHRUSTACCSETPOINT"))

(in-package "ROS")
;;//! \htmlinclude VehicleThrustAccSetpoint.msg.html


(defclass px4_msgs::VehicleThrustAccSetpoint
  :super ros::object
  :slots (_timestamp _thrust_acc_sp _rates_sp _model_ff _lead_information ))

(defmethod px4_msgs::VehicleThrustAccSetpoint
  (:init
   (&key
    ((:timestamp __timestamp) 0)
    ((:thrust_acc_sp __thrust_acc_sp) 0.0)
    ((:rates_sp __rates_sp) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:model_ff __model_ff) 0.0)
    ((:lead_information __lead_information) 0.0)
    )
   (send-super :init)
   (setq _timestamp (round __timestamp))
   (setq _thrust_acc_sp (float __thrust_acc_sp))
   (setq _rates_sp __rates_sp)
   (setq _model_ff (float __model_ff))
   (setq _lead_information (float __lead_information))
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:thrust_acc_sp
   (&optional __thrust_acc_sp)
   (if __thrust_acc_sp (setq _thrust_acc_sp __thrust_acc_sp)) _thrust_acc_sp)
  (:rates_sp
   (&optional __rates_sp)
   (if __rates_sp (setq _rates_sp __rates_sp)) _rates_sp)
  (:model_ff
   (&optional __model_ff)
   (if __model_ff (setq _model_ff __model_ff)) _model_ff)
  (:lead_information
   (&optional __lead_information)
   (if __lead_information (setq _lead_information __lead_information)) _lead_information)
  (:serialization-length
   ()
   (+
    ;; uint64 _timestamp
    8
    ;; float32 _thrust_acc_sp
    4
    ;; float32[3] _rates_sp
    (* 4    3)
    ;; float32 _model_ff
    4
    ;; float32 _lead_information
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint64 _timestamp
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _timestamp (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _timestamp) (= (length (_timestamp . bv)) 2)) ;; bignum
              (write-long (ash (elt (_timestamp . bv) 0) 0) s)
              (write-long (ash (elt (_timestamp . bv) 1) -1) s))
             ((and (class _timestamp) (= (length (_timestamp . bv)) 1)) ;; big1
              (write-long (elt (_timestamp . bv) 0) s)
              (write-long (if (>= _timestamp 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _timestamp s)(write-long (if (>= _timestamp 0) 0 #xffffffff) s)))
     ;; float32 _thrust_acc_sp
       (sys::poke _thrust_acc_sp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[3] _rates_sp
     (dotimes (i 3)
       (sys::poke (elt _rates_sp i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _model_ff
       (sys::poke _model_ff (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lead_information
       (sys::poke _lead_information (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint64 _timestamp
     
#+(or :alpha :irix6 :x86_64)
      (setf _timestamp (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _timestamp (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float32 _thrust_acc_sp
     (setq _thrust_acc_sp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[3] _rates_sp
   (dotimes (i (length _rates_sp))
     (setf (elt _rates_sp i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _model_ff
     (setq _model_ff (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lead_information
     (setq _lead_information (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get px4_msgs::VehicleThrustAccSetpoint :md5sum-) "5872a8a2a7714255ea85f043e820058d")
(setf (get px4_msgs::VehicleThrustAccSetpoint :datatype-) "px4_msgs/VehicleThrustAccSetpoint")
(setf (get px4_msgs::VehicleThrustAccSetpoint :definition-)
      "
uint64 timestamp        # time since system start (microseconds)



float32 thrust_acc_sp          # thrust acceleration setpoint along the thrust axis, like amostly Z body axis.

float32[3] rates_sp

float32 model_ff
float32 lead_information


")



(provide :px4_msgs/VehicleThrustAccSetpoint "5872a8a2a7714255ea85f043e820058d")


