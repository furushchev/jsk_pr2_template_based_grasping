;; test.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "tabletop_object_detector")
(ros::load-ros-manifest "object_manipulation_msgs")

(ros::roseus "hoge")

(require "package://pr2eus/pr2-interface.l")

(pr2-init)

(make-irtviewer)

(defun grasp-object (&optional (target-pose (make-coords :pos #f(600 0 800))))
  (let (targets target (retry-max 3))
    ;; tabletop object detection
    (while (and (null targets) (>= (dec retry-max) 0))
           (setq req (instance tabletop_object_detector::tabletopsegmentationrequest :init))
           (setq res (ros::service-call "tabletop_segmentation" req))
           (setq table (ros::tf-pose-stamped->coords (send res :table :pose)))
           (setq targets (send res :clusters)))
    (ros::ros-info "~A objects found on table ~A" (length targets) table)
    (setq res0 res)

    ;; select nearest cluster to target-pose
    (let ((min-diff-pos *inf*) eus-pc diff-pos target-centroid)
      (dolist (c targets)
        (setq eus-pc (make-eus-pointcloud-from-ros-msg1 c))
        (setq diff-pos (norm (v- (send eus-pc :centroid) (send target-pose :worldpos))))
        (when (< diff-pos min-diff-pos)
          (setq min-diff-pos diff-pos)
          (setq target-centroid (send eus-pc :centroid))
          (setq target c)))
      (ros::ros-info "target found at ~A" target-centroid))

    (when (boundp '*irtviewer*)
      (objects (list (make-eus-pointcloud-from-ros-msg1 target))))

    ;; send request to grasp planner
    (ros::ros-info "sending request to grasp planner")
    (setq req (instance object_manipulation_msgs::GraspPlanningRequest :init))
    (send req :target :cluster target)
    (send req :target :region :roi_box_pose (send res :table :pose))
    (setq res (ros::service-call "pr2_template_grasp_planner" req))
    (case (send res :error_code)
      (object_manipulation_msgs::graspplanningerrorcode::*tf_error*
       (ros::ros-error "planner response: tf_error")
       (return-from grasp-object nil))
      (object_manipulation_msgs::graspplanningerrorcode::*other_error*
       (ros::ros-error "planner response: other_error")
       (return-from grasp-object nil))
      (t
       (ros::ros-info "planner response: success")))

    (when (boundp '*irtviewer*)
      (objects (append (send *irtviewer* :objects)
                       (mapcar #'(lambda (x)
                                   (send (make-cube 10 10 10) :transform
                                         (ros::tf-pose->coords x)))
                               (send-all (send res :grasps) :grasp_pose)))))

    (dolist (grasp (send res :grasps))
      (let ((avs-tms (solve-grasp-object-motion grasp)))
        (when avs-tms
          (send *ri* :angle-vector (grasp-standby-motion) 5000)
          (send *ri* :wait-interpolation)

          (send *ri* :stop-grasp :rarm :wait t)

          (send *ri* :angle-vector-sequence
                (car avs-tms) (cdr avs-tms))
          (send *ri* :wait-interpolation)
          (return-from grasp-object (send *ri* :start-grasp :rarm)))))
    ))

(defun grasp-standby-motion ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :r_shoulder_pan_joint :joint-angle -57.1453)
  (send *pr2* :r_shoulder_lift_joint :joint-angle -20.2543)
  (send *pr2* :r_upper_arm_roll_joint :joint-angle -72.9576)
  (send *pr2* :r_elbow_flex_joint :joint-angle -93.7955)
  (send *pr2* :r_forearm_roll_joint :joint-angle -38.737)
  (send *pr2* :r_wrist_flex_joint :joint-angle -45.3103)
  (send *pr2* :r_wrist_roll_joint :joint-angle -383.741)
  (send *pr2* :angle-vector))

(defun solve-grasp-object-motion (grasp)
  (let ((grasp-cds (ros::tf-pose->coords (send grasp :grasp_pose)))
        (approach-factor 0.8)
        avs tms)
    (ros::ros-info "solving motion with grasp pose ~A" grasp-cds)

    ;; pre grasp pose
    (unless (send *pr2* :rarm :inverse-kinematics grasp-cds
                              :warn-p nil)
      (ros::ros-error "failed to solve ik for pre grasp")
      (return-from solve-grasp-object-motion nil))
    (apply-joint_state (send grasp :pre_grasp_posture) *pr2*)
    (push (send *pr2* :angle-vector) avs)
    (push 5000 tms)

    ;; approach
    (unless (send *pr2* :rarm :move-end-pos
                  (float-vector
                   (* 1000.0 approach-factor (send grasp :desired_approach_distance)) 0 0)
                  :warn-p nil)
      (ros::ros-error "failed to solve ik for approach")
      (return-from solve-grasp-object-motion nil))
    (push (send *pr2* :angle-vector) avs)
    (push 3000 tms)

    (return-from solve-grasp-object-motion
      (cons (reverse avs) (reverse tms)))))

(grasp-object)
