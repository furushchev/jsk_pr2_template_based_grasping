;; utils.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "jsk_recognition_msgs")

(defvar *base-frame-id* "/base_footprint")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; utilities
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun check-service-available (srvname &key (timeout 2))
  (unless (and (ros::service-exists srvname)
               (ros::wait-for-service srvname timeout))
    (ros::ros-error "service ~A is not available" srvname)
    (return-from check-service-available nil))
  t)

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
                                  :x (* 0.001 (x-of-cube c))
                                  :y (* 0.001 (y-of-cube c))
                                  :z (* 0.001 (z-of-cube c)))))

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
    (when (or (substringp "+" astr) (substringp "-" astr))
      (when (substringp "X" astr)
        (send ret :translate (float-vector (* (if (substringp "+" astr) 1 -1) mm 0.5) 0 0)))
      (when (substringp "Y" astr)
        (send ret :translate (float-vector 0 (* (if (substringp "+" astr) 1 -1) mm 0.5) 0)))
      (when (substringp "Z" astr)
        (send ret :translate (float-vector 0 0 (* (if (substringp "+" astr) 1 -1) mm 0.5)))))
    ret))

(provide :utils) ;; end of utils.l
