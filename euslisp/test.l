;; test.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "test_srv")
(ros::load-ros-manifest "pr2_template_based_grasping")
(ros::load-ros-manifest "grasp_template_planning")
(ros::load-ros-manifest "object_manipulation_msgs")
(ros::load-ros-manifest "pr2_gripper_sensor_msgs")
(ros::load-ros-manifest "pr2_controllers_msgs")

;;(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defun test-srv ()
  (let ((srv-name "/pr2_template_grasp_planner")
        (req (instance object_manipulation_msgs::GraspPlanningRequest :init))
        res)
    (ros::wait-for-service srv-name 10)
    (setq res (ros::service-call srv-name req))
    (print-ros-msg res)))

(test-srv)

(provide :test) ;; end of test.l
