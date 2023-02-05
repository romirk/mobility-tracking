;; Auto-generated. Do not edit!


(when (boundp 'sensorbox::AQI)
  (if (not (find-package "SENSORBOX"))
    (make-package "SENSORBOX"))
  (shadow 'AQI (find-package "SENSORBOX")))
(unless (find-package "SENSORBOX::AQI")
  (make-package "SENSORBOX::AQI"))

(in-package "ROS")
;;//! \htmlinclude AQI.msg.html


(defclass sensorbox::AQI
  :super ros::object
  :slots (_pm10 _pm25 _pm50 _pm100 _tmp _hum _co2 ))

(defmethod sensorbox::AQI
  (:init
   (&key
    ((:pm10 __pm10) 0.0)
    ((:pm25 __pm25) 0.0)
    ((:pm50 __pm50) 0.0)
    ((:pm100 __pm100) 0.0)
    ((:tmp __tmp) 0.0)
    ((:hum __hum) 0.0)
    ((:co2 __co2) 0.0)
    )
   (send-super :init)
   (setq _pm10 (float __pm10))
   (setq _pm25 (float __pm25))
   (setq _pm50 (float __pm50))
   (setq _pm100 (float __pm100))
   (setq _tmp (float __tmp))
   (setq _hum (float __hum))
   (setq _co2 (float __co2))
   self)
  (:pm10
   (&optional __pm10)
   (if __pm10 (setq _pm10 __pm10)) _pm10)
  (:pm25
   (&optional __pm25)
   (if __pm25 (setq _pm25 __pm25)) _pm25)
  (:pm50
   (&optional __pm50)
   (if __pm50 (setq _pm50 __pm50)) _pm50)
  (:pm100
   (&optional __pm100)
   (if __pm100 (setq _pm100 __pm100)) _pm100)
  (:tmp
   (&optional __tmp)
   (if __tmp (setq _tmp __tmp)) _tmp)
  (:hum
   (&optional __hum)
   (if __hum (setq _hum __hum)) _hum)
  (:co2
   (&optional __co2)
   (if __co2 (setq _co2 __co2)) _co2)
  (:serialization-length
   ()
   (+
    ;; float32 _pm10
    4
    ;; float32 _pm25
    4
    ;; float32 _pm50
    4
    ;; float32 _pm100
    4
    ;; float32 _tmp
    4
    ;; float32 _hum
    4
    ;; float32 _co2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pm10
       (sys::poke _pm10 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pm25
       (sys::poke _pm25 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pm50
       (sys::poke _pm50 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pm100
       (sys::poke _pm100 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tmp
       (sys::poke _tmp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _hum
       (sys::poke _hum (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _co2
       (sys::poke _co2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pm10
     (setq _pm10 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pm25
     (setq _pm25 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pm50
     (setq _pm50 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pm100
     (setq _pm100 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tmp
     (setq _tmp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _hum
     (setq _hum (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _co2
     (setq _co2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get sensorbox::AQI :md5sum-) "40b6414f047f4614963c31c7b66060cd")
(setf (get sensorbox::AQI :datatype-) "sensorbox/AQI")
(setf (get sensorbox::AQI :definition-)
      "float32 pm10
float32 pm25
float32 pm50
float32 pm100
float32 tmp
float32 hum
float32 co2
")



(provide :sensorbox/AQI "40b6414f047f4614963c31c7b66060cd")


