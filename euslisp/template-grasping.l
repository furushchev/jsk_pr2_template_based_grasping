;; template-grasping.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "template_grasping_client")

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "object_manipulation_msgs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :tabletop-object-detection "tabletop-object-detection.l")

(defparameter *debug-view* t)
(defparameter *table-bounding-box-topic* "/cluster_decomposer/boxes")
(defparameter *tabletop-bounding-box-topic* "/cluster_decomposer_final/boxes")
(defparameter *object-bounding-box-topic* "/object_bounding_box")
(defparameter *object-bounding-box-check-topic* "/tabletop_object_detector/output/box_array")
(defparameter *object-cloud-topic* "tabletop_object_cloud/output")
(defparameter *template-grasp-service* "/pr2_template_grasp_planner")
(defparameter *base-frame-id* "/base_footprint")
(defparameter *object-margin* 50)

(defun template-grasp-planning (target-pc table)
  (unless (check-service-available *template-grasp-service*)
    (return-from template-grasp-planning nil))
  (let* ((gpreq (instance object_manipulation_msgs::GraspPlanningRequest :init))
        gpres)
    (setq gpres
          (ros::service-call *template-grasp-service* gpreq))
    gpres))


(defun init ()
  (pr2-init)
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))
  (when *debug-view*
    (objects (list *pr2*))))

(init)
(setq *table* (table-detection))
(setq *object* (tabletop-object-detection (make-coords :pos #f(600 0 800)) *table*))
(setq *plan* (template-grasp-planning *table* *object*))
(print-ros-msg *plan*)
