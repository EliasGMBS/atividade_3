#!/usr/bin/env python3
import rospy
import math
import csv
import os
import tf.transformations as tft
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates

class AMCLEvaluator:
    def __init__(self):
        rospy.init_node('amcl_evaluator', anonymous=True)
        self.map_name = rospy.get_param('~map_name', 'unknown')
        self.results_dir = '/home/elias/atividade_3/results'
        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir)
            
        self.amcl_pose = None
        self.gt_pose = None
        
        self.data = []
        self.t0 = None
        
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gt_cb)
        
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_cb)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(f"AMCL Evaluator started for map: {self.map_name}")
        
    def amcl_cb(self, msg):
        self.amcl_pose = msg
        
    def gt_cb(self, msg):
        try:
            idx = msg.name.index('husky')
            self.gt_pose = msg.pose[idx]
        except ValueError:
            pass
            
    def timer_cb(self, event):
        if self.amcl_pose and self.gt_pose:
            if self.t0 is None:
                self.t0 = rospy.Time.now().to_sec()
                
            t = rospy.Time.now().to_sec() - self.t0
            
            # AMCL
            amcl_x = self.amcl_pose.pose.pose.position.x
            amcl_y = self.amcl_pose.pose.pose.position.y
            q_amcl = self.amcl_pose.pose.pose.orientation
            _, _, amcl_yaw = tft.euler_from_quaternion([q_amcl.x, q_amcl.y, q_amcl.z, q_amcl.w])
            
            # GT
            gt_x = self.gt_pose.position.x
            gt_y = self.gt_pose.position.y
            q_gt = self.gt_pose.orientation
            _, _, gt_yaw = tft.euler_from_quaternion([q_gt.x, q_gt.y, q_gt.z, q_gt.w])
            
            # Errors
            pos_error = math.sqrt((amcl_x - gt_x)**2 + (amcl_y - gt_y)**2)
            
            # Normalize yaw error to [-pi, pi]
            yaw_error = amcl_yaw - gt_yaw
            while yaw_error > math.pi: yaw_error -= 2 * math.pi
            while yaw_error < -math.pi: yaw_error += 2 * math.pi
                
            self.data.append({
                'time': t,
                'pos_error': pos_error,
                'yaw_error': yaw_error
            })
            
    def on_shutdown(self):
        if not self.data:
            rospy.logwarn("No data collected.")
            return
            
        n = len(self.data)
        pos_errors = [d['pos_error'] for d in self.data]
        yaw_errors = [abs(d['yaw_error']) for d in self.data]
        
        mean_pos_error = sum(pos_errors) / n
        rmse = math.sqrt(sum(e**2 for e in pos_errors) / n)
        max_pos_error = max(pos_errors)
        final_pos_error = pos_errors[-1]
        
        mean_yaw_error_deg = math.degrees(sum(yaw_errors) / n)
        max_yaw_error_deg = math.degrees(max(yaw_errors))
        
        rospy.loginfo(f"--- AMCL Evaluation Results: {self.map_name} ---")
        rospy.loginfo(f"Mean Pos Error: {mean_pos_error:.4f} m")
        rospy.loginfo(f"RMSE Pos: {rmse:.4f} m")
        rospy.loginfo(f"Max Pos Error: {max_pos_error:.4f} m")
        rospy.loginfo(f"Final Pos Error: {final_pos_error:.4f} m")
        rospy.loginfo(f"Mean Yaw Error: {mean_yaw_error_deg:.4f} deg")
        rospy.loginfo(f"Max Yaw Error: {max_yaw_error_deg:.4f} deg")
        
        csv_file = os.path.join(self.results_dir, f'amcl_evaluation_{self.map_name}.csv')
        with open(csv_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['Metric', 'Value'])
            writer.writerow(['Mean Pos Error (m)', mean_pos_error])
            writer.writerow(['RMSE Pos (m)', rmse])
            writer.writerow(['Max Pos Error (m)', max_pos_error])
            writer.writerow(['Final Pos Error (m)', final_pos_error])
            writer.writerow(['Mean Yaw Error (deg)', mean_yaw_error_deg])
            writer.writerow(['Max Yaw Error (deg)', max_yaw_error_deg])

if __name__ == '__main__':
    try:
        AMCLEvaluator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
