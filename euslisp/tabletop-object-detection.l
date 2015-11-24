;; tabletop-object-detection.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "jsk_recognition_msgs")

(require :table-detection "table-detection.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defparameter *debug-view* t)
(defparameter *table-bounding-box-topic* "/cluster_decomposer/boxes")
(defparameter *tabletop-bounding-box-topic* "/cluster_decomposer_final/boxes")
(defparameter *object-bounding-box-topic* "/object_bounding_box")
(defparameter *object-bounding-box-check-topic* "/tabletop_object_detector/output/box_array")
(defparameter *object-cloud-topic* "tabletop_object_cloud/output")
(defparameter *base-frame-id* "/base_footprint")
(defparameter *object-margin* 50)

(defun tabletop-bbox-cb (target-pose msg)
  (unless
      (send *tfl* :wait-for-transform *base-frame-id* (send msg :header :frame_id) (ros::time 0) 10)
    (return-from table-bbox-cb nil))
  (let ((base->box-parent
          (send *tfl* :lookup-transform *base-frame-id* (send msg :header :frame_id) (ros::time 0)))
        bboxes)
    (setq bboxes (boundingboxarray->cubes msg))
    (send-all bboxes :transform base->box-parent *pr2*)
    (when *debug-view*
      (objects (append (list *pr2*) bboxes))
      (ros::ros-info "all bbox: ~A" (send-all bboxes :centroid))
      (unix:sleep 1))
    (setq bboxes
          (remove-if-not #'(lambda (c)
                             (or
                              (eq :inside (send *tabletop-bbox* :insidep (send c :centroid)))
                              (send *tabletop-bbox* :intersectp c)))
                         bboxes))
    (when *debug-view*
      (objects (append (list *pr2*) bboxes))
      (ros::ros-info "bbox on table: ~A" (send-all bboxes :centroid))
      (unix:sleep 1))
    (setq *target-bbox*
          (cube+
           (car (sort bboxes #'<=
                      #'(lambda (x)
                          (norm (send target-pose :difference-position x)))))
           *object-margin*
           :xyz))
    (when *debug-view*
      (objects (list *pr2* *target-bbox*))
      (ros::ros-info "nearest bbox: ~A" (send *target-bbox* :centroid)))
    *target-bbox*
    ))

(defun target-object-cloud-cb (msg)
  (ros::ros-info "received target object cloud ~A" *attention-changed*)
  (when *attention-changed*
    (setq *target-object-cloud* msg)))

(defun target-bbox-cb (msg)
  (let* ((box (car (boundingboxarray->cubes msg)))
         (volume-diff (abs (- (send *target-bbox* :volume) (send box :volume)))))
    (ros::ros-info "bbox: ~A ~A ~A" (send *target-bbox* :volume) (send box :volume) volume-diff)
    (setq *attention-changed* (< volume-diff 1.0))))


(defun tabletop-object-detection (target-pose &optional table)
  (unless (ros::ok)
    (ros::roseus "tabletop_object_detection"))
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))
  (unless (boundp '*ri*)
    (pr2-init))
  (unless table
    (setq table (table-detection)))

  (setq *target-bbox* nil)
  (setq *target-object-cloud* nil)
  (setq *attention-changed* nil)
  (setq *tabletop-bbox* (cube+ table 500))
  (when *debug-view*
    (objects (list *pr2* *tabletop-bbox*)))
  (ros::subscribe *tabletop-bounding-box-topic*
                  jsk_recognition_msgs::boundingboxarray
                  #'tabletop-bbox-cb target-pose)
  (ros::rate 1)
  (while (not *target-bbox*)
   (ros::ros-info "waiting object bounding boxes...")
   (ros::spin-once)
   (ros::sleep))
  (ros::unsubscribe *tabletop-bounding-box-topic*)
  (ros::advertise *object-bounding-box-topic* jsk_recognition_msgs::boundingbox)
  (unix:sleep 1)
  (ros::subscribe *object-cloud-topic*
                  sensor_msgs::PointCloud2
                  #'target-object-cloud-cb)
  (ros::subscribe *object-bounding-box-check-topic*
                  jsk_recognition_msgs::boundingboxarray
                  #'target-bbox-cb)
  (ros::publish *object-bounding-box-topic*
                (cube->boundingbox *target-bbox*))
  (while (not *target-object-cloud*)
    (ros::ros-info "waiting target object cloud...")
    (ros::spin-once)
    (ros::sleep))
  (ros::unsubscribe *object-cloud-topic*)
  (ros::unsubscribe *object-bounding-box-check-topic*)
  *target-object-cloud*)
  ;; (let ((eus-pc
  ;;         (make-eus-pointcloud-from-ros-msg *target-object-cloud*)))
  ;;   (send eus-pc :name (send *target-object-cloud* :name))
  ;;   eus-pc))
(warn ";; tabletop-object-detection~%")

(provide :tabletop-object-detection) ;; end of tabletop-object-detection.l
