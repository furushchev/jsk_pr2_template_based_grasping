;; template-grasping.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "template_grasping_client")

(ros::load-ros-manifest "jsk_recognition_msgs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defparameter *debug-view* t)
(defparameter *table-bounding-box-topic* "/cluster_decomposer/boxes")
(defparameter *tabletop-bounding-box-topic* "/cluster_decomposer_final/boxes")
(defparameter *base-frame-id* "/base_footprint")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; geometry
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun cube-scale (c factor &optional (axis :xyz))
  (let ((astr (string axis)))
    (make-cube (* (if (substringp "X" astr) factor 1) (x-of-cube c))
               (* (if (substringp "Y" astr) factor 1) (y-of-cube c))
               (* (if (substringp "Z" astr) factor 1) (z-of-cube c))
               :coords c)))

(defun cube+ (c mm &optional (axis :z+))
  (let* ((astr (string axis))
         (ret
           (make-cube (+ (if (substringp "X" astr) mm 0) (x-of-cube c))
                      (+ (if (substringp "Y" astr) mm 0) (y-of-cube c))
                      (+ (if (substringp "Z" astr) mm 0) (z-of-cube c))
                      :coords c)))
    (when (substringp "X" astr)
      (send ret :translate (float-vector (* (if (substringp "+" astr) 1 -1) mm 0.5) 0 0)))
    (when (substringp "Y" astr)
      (send ret :translate (float-vector 0 (* (if (substringp "+" astr) 1 -1) mm 0.5) 0)))
    (when (substringp "Z" astr)
      (send ret :translate (float-vector 0 0 (* (if (substringp "+" astr) 1 -1) mm 0.5))))
    ret))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; conversion
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun boundingbox->cube (msg &key color (alpha 1.0))
  (let* ((dim (ros::tf-point->pos (send msg :dimensions)))
         (c (make-cube (elt dim 0) (elt dim 1) (elt dim 2)
                       :coords (ros::tf-pose->coords (send msg :pose)))))
    (setf (get c :face-color)
          (if color color (color-category20 (random 20))))
    (gl::transparent c alpha)
    c))

(defun boundingboxarray->cubes (msg &key color (alpha 1.0))
  (mapcar #'(lambda (x) (boundingbox->cube x :color color :alpha alpha)) (send msg :boxes)))

(defun cube->boundingbox (c &key (frame_id *base-frame-id*))
  (instance jsk_recognition_msgs::boundingbox :init
            :header (instance std_msgs::header :init
                              :stamp (ros::time-now)
                              :frame_id frame_id)
            :pose (ros::coords->tf-pose c)
            :dimensions (instance geometry_msgs::vector3 :init
                                  :x (x-of-cube c)
                                  :y (y-of-cube c)
                                  :z (z-of-cube c))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; table estimation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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
        planes)
    (setq planes (boundingboxarray->cubes msg))
    (send-all planes :transform base->box-parent *pr2*)
    (push (find-table-candidate planes) *table-candidates*)
    (when *debug-view*
      (objects (list *pr2* (car *table-candidates*))))))

(defun table-detection ()
  (setq *table-candidates* nil)
  (ros::subscribe *table-bounding-box-topic*
                  jsk_recognition_msgs::boundingboxarray
                  #'table-bbox-cb)
  (ros::rate 1)
  (while (< (length *table-candidates*) 3)
   (ros::spin-once)
   (ros::sleep))
  (ros::unsubscribe *table-bounding-box-topic*)
  (car *table-candidates*))
  ;; (sort *table-candidates* #'<=
  ;;       #'(lambda (x) (send x :difference-rotation
  ;;                           (make-coords :rpy (list 0 0 0)))))
  ;; (make-coords :pos (vector-mean (send-all *table-candidates* :worldpos))
  ;;              :rpy (car (send (car *table-candidates*) :rpy-angle))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; tabletop object estimation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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
      (unix:sleep 1))
    (setq bboxes
          (remove-if-not #'(lambda (c)
                             (send *tabletop-bbox* :intersectp c))
                         bboxes))
    (when *debug-view*
      (objects (append (list *pr2*) bboxes))
      (unix:sleep 1))
    (setq *target-bbox*
          (cube-scale
           (car (sort bboxes #'<=
                      #'(lambda (x)
                          (norm (send target-pose :difference-position x)))))
           1.3))
    (when *debug-view*
      (objects (list *pr2* *target-bbox*)))
    *target-bbox*
    ))

(defun tabletop-object-detection (target-pose)
  (setq *tabletop-bbox* (cube+ (car *table-candidates*) 500))
  (gl::transparent *tabletop-bbox* 0.6)
  (setq *target-bbox* nil)
  (when *debug-view*
    (objects (list *pr2* *tabletop-bbox*)))
  (ros::subscribe *tabletop-bounding-box-topic*
                  jsk_recognition_msgs::boundingboxarray
                  #'tabletop-bbox-cb target-pose)
  (ros::rate 1)
  (while (not *target-bbox*)
   (ros::spin-once)
   (ros::sleep))
  (ros::unsubscribe *tabletop-bounding-box-topic*))

(defun init ()
  (pr2-init)
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init))
    (send *tfl* :wait-for-server))
  (when *debug-view*
    (objects (list *pr2*)))
  )

(init)
(table-detection)
(unix:sleep 2)
(tabletop-object-detection (make-coords :pos #f(600 0 800)))

(provide :template-grasping) ;; end of template-grasping.l
