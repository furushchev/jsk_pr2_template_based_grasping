;; tabletop-object-detection-server.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "grasp_template_planning")

(require :tabletop-object-detection "tabletop-object-detection.l")

(defun tabletop-service-cb (req)
  (let* ((res (instance grasp_template_planning::tabletopobjectresponse :init))
         (table (table-detection))
         (target-cloud (tabletop-object-detection
                        (make-coords :pos #f(600 0 800))
                        table)))
    (send res :table_pose (ros::coords->tf-pose-stamped table *base-frame-id*))
    (send res :target_cloud target-cloud)
    res))

(ros::roseus "tabletop_object_detection_server")
(ros::advertise-service "tabletop_object_detection"
                        grasp_template_planning::tabletopobject
                        #'tabletop-service-cb)
(ros::rate 1)
(ros::spin)
