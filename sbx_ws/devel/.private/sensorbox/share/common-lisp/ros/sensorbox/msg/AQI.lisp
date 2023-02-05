; Auto-generated. Do not edit!


(cl:in-package sensorbox-msg)


;//! \htmlinclude AQI.msg.html

(cl:defclass <AQI> (roslisp-msg-protocol:ros-message)
  ((pm10
    :reader pm10
    :initarg :pm10
    :type cl:float
    :initform 0.0)
   (pm25
    :reader pm25
    :initarg :pm25
    :type cl:float
    :initform 0.0)
   (pm50
    :reader pm50
    :initarg :pm50
    :type cl:float
    :initform 0.0)
   (pm100
    :reader pm100
    :initarg :pm100
    :type cl:float
    :initform 0.0)
   (tmp
    :reader tmp
    :initarg :tmp
    :type cl:float
    :initform 0.0)
   (hum
    :reader hum
    :initarg :hum
    :type cl:float
    :initform 0.0)
   (co2
    :reader co2
    :initarg :co2
    :type cl:float
    :initform 0.0))
)

(cl:defclass AQI (<AQI>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AQI>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AQI)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensorbox-msg:<AQI> is deprecated: use sensorbox-msg:AQI instead.")))

(cl:ensure-generic-function 'pm10-val :lambda-list '(m))
(cl:defmethod pm10-val ((m <AQI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensorbox-msg:pm10-val is deprecated.  Use sensorbox-msg:pm10 instead.")
  (pm10 m))

(cl:ensure-generic-function 'pm25-val :lambda-list '(m))
(cl:defmethod pm25-val ((m <AQI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensorbox-msg:pm25-val is deprecated.  Use sensorbox-msg:pm25 instead.")
  (pm25 m))

(cl:ensure-generic-function 'pm50-val :lambda-list '(m))
(cl:defmethod pm50-val ((m <AQI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensorbox-msg:pm50-val is deprecated.  Use sensorbox-msg:pm50 instead.")
  (pm50 m))

(cl:ensure-generic-function 'pm100-val :lambda-list '(m))
(cl:defmethod pm100-val ((m <AQI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensorbox-msg:pm100-val is deprecated.  Use sensorbox-msg:pm100 instead.")
  (pm100 m))

(cl:ensure-generic-function 'tmp-val :lambda-list '(m))
(cl:defmethod tmp-val ((m <AQI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensorbox-msg:tmp-val is deprecated.  Use sensorbox-msg:tmp instead.")
  (tmp m))

(cl:ensure-generic-function 'hum-val :lambda-list '(m))
(cl:defmethod hum-val ((m <AQI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensorbox-msg:hum-val is deprecated.  Use sensorbox-msg:hum instead.")
  (hum m))

(cl:ensure-generic-function 'co2-val :lambda-list '(m))
(cl:defmethod co2-val ((m <AQI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensorbox-msg:co2-val is deprecated.  Use sensorbox-msg:co2 instead.")
  (co2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AQI>) ostream)
  "Serializes a message object of type '<AQI>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pm10))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pm25))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pm50))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pm100))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tmp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hum))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'co2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AQI>) istream)
  "Deserializes a message object of type '<AQI>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pm10) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pm25) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pm50) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pm100) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tmp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hum) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'co2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AQI>)))
  "Returns string type for a message object of type '<AQI>"
  "sensorbox/AQI")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AQI)))
  "Returns string type for a message object of type 'AQI"
  "sensorbox/AQI")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AQI>)))
  "Returns md5sum for a message object of type '<AQI>"
  "40b6414f047f4614963c31c7b66060cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AQI)))
  "Returns md5sum for a message object of type 'AQI"
  "40b6414f047f4614963c31c7b66060cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AQI>)))
  "Returns full string definition for message of type '<AQI>"
  (cl:format cl:nil "float32 pm10~%float32 pm25~%float32 pm50~%float32 pm100~%float32 tmp~%float32 hum~%float32 co2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AQI)))
  "Returns full string definition for message of type 'AQI"
  (cl:format cl:nil "float32 pm10~%float32 pm25~%float32 pm50~%float32 pm100~%float32 tmp~%float32 hum~%float32 co2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AQI>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AQI>))
  "Converts a ROS message object to a list"
  (cl:list 'AQI
    (cl:cons ':pm10 (pm10 msg))
    (cl:cons ':pm25 (pm25 msg))
    (cl:cons ':pm50 (pm50 msg))
    (cl:cons ':pm100 (pm100 msg))
    (cl:cons ':tmp (tmp msg))
    (cl:cons ':hum (hum msg))
    (cl:cons ':co2 (co2 msg))
))
