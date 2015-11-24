;; table-detection.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "jsk_recognition_msgs")

(require :utils "utils.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defparameter *debug-view* t)
(defparameter *table-bounding-box-topic* "/cluster_decomposer/boxes")
(defparameter *tabletop-bounding-box-topic* "/cluster_decomposer_final/boxes")
(defparameter *base-frame-id* "/base_footprint")

(defun find-table-candidate (planes)
  (car
   (sort
    (remove-if-not
     #'(lambda (x)
         (and
          (> (send x :volume) 1e+7)
          (> (elt (send x :worldpos) 2) 500)))
     planes)
    #'<= #'(lambda (x)
             (norm (v*
                    (float-vector 0 0 1)
                    (send x :z-axis)))))))

(defun table-bbox-cb (msg)
  (unless
      (send *tfl* :wait-for-transform *base-frame-id* (send msg :header :frame_id) (ros::time 0) 10)
    (return-from table-bbox-cb nil))
  (let ((base->box-parent
          (send *tfl* :lookup-transform *base-frame-id* (send msg :header :frame_id) (ros::time 0)))
        planes cand)
    (setq planes (boundingboxarray->cubes msg))
    (send-all planes :transform base->box-parent *pr2*)
    (setq cand (find-table-candidate planes))
    (unless cand
      (ros::ros-warn "could not find table candidate")
      (return-from table-bbox-cb nil))
    (push cand *table-candidates*)
    (when *debug-view*
      (objects (list *pr2* (car *table-candidates*))))))

(defun table-detection ()
  (unless (ros::ok)
    (ros::roseus "table_detection"))
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))
  (unless (boundp '*ri*)
    (pr2-init))
  (setq *table-candidates* nil)
  (ros::subscribe *table-bounding-box-topic*
                  jsk_recognition_msgs::boundingboxarray
                  #'table-bbox-cb)
  (ros::rate 1)
  (while (< (length *table-candidates*) 3)
   (ros::ros-info "waiting table...")
   (ros::spin-once)
   (ros::sleep))
  (ros::unsubscribe *table-bounding-box-topic*)
  (car *table-candidates*))
(warn ";; table-detection~%")

(provide :table-detection) ;; end of table-detection.l
