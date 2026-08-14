import math, unittest
import numpy as np
from suturing_runtime.contract import approach_goal_matrix,bounded_pose_step,inside_workspace,matrix_to_quaternion_xyzw,pose_matrix
class ContractTest(unittest.TestCase):
    def test_quaternion_round_trip(self):
        pose=pose_matrix([.1,-.2,.3],[.1,.2,.3,.9]); q=matrix_to_quaternion_xyzw(pose[:3,:3])
        np.testing.assert_allclose(pose_matrix(pose[:3,3],q),pose,atol=1e-10)
    def test_bounded_step(self):
        current=np.eye(4); target=np.eye(4); target[0,3]=.01
        c,s=math.cos(math.pi/4),math.sin(math.pi/4); target[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
        step,distance,angle=bounded_pose_step(current,target,.001,math.radians(5))
        self.assertAlmostEqual(distance,.01); self.assertAlmostEqual(math.degrees(angle),45.); self.assertAlmostEqual(step[0,3],.001)
    def test_goal_is_finite(self):
        goal=approach_goal_matrix(np.eye(4),12.5,.007); self.assertTrue(np.isfinite(goal).all()); self.assertAlmostEqual(np.linalg.det(goal[:3,:3]),1.,places=8)
    def test_workspace(self):
        self.assertTrue(inside_workspace([0,0,.1],[-.2]*3,[.2]*3)); self.assertFalse(inside_workspace([.3,0,.1],[-.2]*3,[.2]*3))
if __name__=="__main__":unittest.main()

