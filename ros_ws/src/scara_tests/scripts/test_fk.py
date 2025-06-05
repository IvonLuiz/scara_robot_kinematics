#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend

received_pose = None

def pose_callback(msg):
    global received_pose
    received_pose = msg

def dh_transform(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def compute_fk(joint1, joint2, joint3):
    T1 = dh_transform(joint1, 1.5, 1, 0)
    T2 = dh_transform(joint2, 0, 1, np.pi)
    T3 = dh_transform(0, joint3, 0, 0)

    T = T1 @ T2 @ T3
    return T[0, 3], T[1, 3], T[2, 3]

def publish_joint_positions(j1, j2, j3):
    pub1.publish(Float64(data=j1))
    pub2.publish(Float64(data=j2))
    pub3.publish(Float64(data=j3))

if __name__ == "__main__":
    rospy.init_node("fk_tester")

    # Publishers
    pub1 = rospy.Publisher("/scara_robot/joint1_position_controller/command", Float64, queue_size=1)
    pub2 = rospy.Publisher("/scara_robot/joint2_position_controller/command", Float64, queue_size=1)
    pub3 = rospy.Publisher("/scara_robot/joint3_position_controller/command", Float64, queue_size=1)

    # Subscriber to ground truth FK
    rospy.Subscriber("/scara_robot/output_pose", Pose, pose_callback)

    rospy.sleep(2.0)

    test_cases = [
        (0.0, 0.0, 0.1),    # 1
        (5.1, 1.2, 0.2),    # 2
        (1.3, -0.2, 0.3),   # 3
        (2.5, 0.5, 0.4),    # 4
        (0.7, -1.4, 0.5),   # 5
        (1.0, 1.5, 0.6),    # 6
        (-2.5, 1.5, 0.7),   # 7
        (-2.8, -0.8, 0.8),  # 8
        (4.2, -0.1, 0.9),   # 9
        (5.6, 1.3, 1.0),    # 10
    ]
    x_diffs, y_diffs, z_diffs = [], [], []
    
    for i, (j1, j2, j3) in enumerate(test_cases):
        rospy.loginfo("-" *50)
        rospy.loginfo(f"[TEST {i+1}] Sending joint values: {j1:.2f}, {j2:.2f}, {j3:.2f}")
        publish_joint_positions(j1, j2, j3)

        x, y, z = compute_fk(j1, j2, j3)

        # Wait for the pose to be received
        rospy.sleep(3.5)

        if received_pose:
            dx = x - received_pose.position.x
            dy = y - received_pose.position.y
            dz = z - received_pose.position.z

            x_diffs.append(dx)
            y_diffs.append(dy)
            z_diffs.append(dz)

            rospy.loginfo(f"DH FK Pose:     x={x:.3f}, y={y:.3f}, z={z:.3f}")
            rospy.loginfo(f"ROS FK Output:  x={received_pose.position.x:.3f}, y={received_pose.position.y:.3f}, z={received_pose.position.z:.3f}")
            rospy.loginfo(f"Difference:     x={dx:.3f}, y={dy:.3f}, z={dz:.3f}")
        else:
            rospy.logwarn("No pose received yet.")
            x_diffs.append(0.0)
            y_diffs.append(0.0)
            z_diffs.append(0.0)

        # Plot differences after all tests
        plt.figure(figsize=(10, 6))
        plt.plot(x_diffs, label='Δx')
        plt.plot(y_diffs, label='Δy')
        plt.plot(z_diffs, label='Δz')
        plt.title("Difference Between DH FK and ROS Pose")
        plt.xlabel("Test Case Index")
        plt.ylabel("Difference (meters)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        
        plt.savefig("fk_differences.png")

