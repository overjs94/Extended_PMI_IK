#!/usr/bin/env python3
"""Optional ROS 1 adapter; publishes solutions, never hardware commands.

Install the research core into the same Python environment as rospy first.
The adapter itself has not been integration-tested in a ROS runtime here.
"""
import threading
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from extended_pmi_ik import RobotModel
from extended_pmi_ik.solver import solve_ik


def main():
    rospy.init_node('extended_pmi_ik')
    model = RobotModel(rospy.get_param('/robot_description'))
    if any(j.kind=='floating' for j in model.joints):
        raise ValueError('ROS JointState bridge requires fixed-base URDF; use the core API for floating objects')
    link = rospy.get_param('~target_link')
    world = rospy.get_param('~world_frame',model.root)
    method = rospy.get_param('~method','reduced-osqp')
    state = {'q':None}
    mutex = threading.Lock()
    publisher = rospy.Publisher('~solution',JointState,queue_size=1)
    def joints(msg):
        values = dict(zip(msg.name,msg.position))
        if all(n in values for n in model.qnames): state['q']=np.array([values[n] for n in model.qnames])
    def target(msg):
        if msg.header.frame_id!=world:
            rospy.logwarn('Target must already be in %s; this adapter does not transform TF frames',world);return
        if state['q'] is None or not mutex.acquire(blocking=False):return
        try:
            p,r=msg.pose.position,msg.pose.orientation
            T=np.eye(4);T[:3,3]=[p.x,p.y,p.z];T[:3,:3]=Rotation.from_quat([r.x,r.y,r.z,r.w]).as_matrix()
            result=solve_ik(model,state['q'].copy(),{link:T},method=method)
            if result['status']!='converged':rospy.logwarn('IK: %s',result['status']);return
            out=JointState();out.header.stamp=rospy.Time.now();out.name=model.qnames;out.position=result['final_q'];publisher.publish(out)
        except Exception as error:rospy.logwarn('IK request failed: %s',error)
        finally:mutex.release()
    rospy.Subscriber('/joint_states',JointState,joints,queue_size=1)
    rospy.Subscriber('~target',PoseStamped,target,queue_size=1)
    rospy.spin()


if __name__=='__main__':main()
