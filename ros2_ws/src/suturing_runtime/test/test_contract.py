import math, unittest
import numpy as np
from suturing_runtime.contract import (FrameContract,approach_goal_matrix,bounded_pose_step,
    frame_contract_issues,inside_workspace,matrix_to_quaternion_xyzw,pose_matrix,
    select_flat_candidate)
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
    def test_frame_contract_reports_all_mismatches(self):
        reference=FrameContract(10,"camera",640,480,"rgb8")
        candidate=FrameContract(11,"wrong",320,240,"mono16")
        issues=frame_contract_issues(reference,candidate,("mono8",))
        self.assertEqual(len(issues),4)
        self.assertTrue(any(item.startswith("stamp:") for item in issues))
    def test_flat_candidate_uses_signed_normal_then_score(self):
        poses=np.repeat(np.eye(4)[None,:,:],3,axis=0)
        poses[1,:3,:3]=np.diag([1.,-1.,-1.])
        c,s=math.cos(math.radians(10)),math.sin(math.radians(10))
        poses[2,:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]]
        selected,angles=select_flat_candidate(poses,[.2,.99,.8],[0,0,1],[0,0,1],20.)
        self.assertEqual(selected,2)
        self.assertAlmostEqual(angles[0],0.)
        self.assertAlmostEqual(angles[1],180.)
if __name__=="__main__":unittest.main()
