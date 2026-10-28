"""Real PyBullet DIRECT probe for the *same* native URDF stop constraints used in GUI.

No UART or real GUI mouse is claimed here. The test checks the physical
limit solver under an imposed joint torque, not a resetJointState simulation
of continuous motion. Starting pose setup via resetJointState is only used
before the physics run and is never part of the GUI's runtime control loop.
"""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "simulation" / "pybullet_ur5" / "src"
sys.path.insert(0, str(SRC))

try:
    import pybullet as p
except ImportError:
    p = None

from stage4_dynamic_closed_loop import URDF, joint_indices
from stage5_qemu_uart_visual_demo import (
    configure_native_urdf_joint_stops,
    GUI_JOINT_STOP_SOLVER_ITERATIONS,
    GUI_JOINT_STOP_NUMERICAL_SLOP_DEG,
)


@unittest.skipIf(p is None, "real pybullet wheel is required")
class NativeBulletJointStopPhysicsTest(unittest.TestCase):
    def test_real_urdf_revolute_stop_resists_external_torque(self):
        connection = p.connect(p.DIRECT)
        self.assertGreaterEqual(connection, 0)
        try:
            p.setGravity(0, 0, 0)
            p.setTimeStep(0.001)
            p.setPhysicsEngineParameter(
                numSolverIterations=GUI_JOINT_STOP_SOLVER_ITERATIONS)
            arm = p.loadURDF(str(URDF), useFixedBase=True)
            joints = joint_indices(p, arm)
            limits = configure_native_urdf_joint_stops(p, arm, joints)
            self.assertEqual(len(limits), 6)
            index = joints[0]
            upper = limits[0][1]
            # Initial fixture only: start near the physical stop.
            p.resetJointState(arm, index, upper - math.radians(1.0))
            for other in joints[1:]:
                p.resetJointState(arm, other, 0.0)
            p.setJointMotorControlArray(
                arm, joints, p.VELOCITY_CONTROL,
                forces=[0.0] * 6)
            maximum = float("-inf")
            for _ in range(450):
                # Actual torque feeds the Bullet solver each 1ms substep.
                p.setJointMotorControl2(
                    arm, index, p.TORQUE_CONTROL, force=150.0)
                p.stepSimulation()
                angle = p.getJointState(arm, index)[0]
                maximum = max(maximum, angle)
            self.assertLessEqual(
                maximum, upper + math.radians(GUI_JOINT_STOP_NUMERICAL_SLOP_DEG),
                f"Native Bullet URDF J1 stop leaked to {math.degrees(maximum):.4f}deg")
            # Release the load and apply opposite torque: the joint must be
            # dynamically movable again; not an artificial frozen/reset pose.
            held = p.getJointState(arm, index)[0]
            for _ in range(150):
                p.setJointMotorControl2(
                    arm, index, p.TORQUE_CONTROL, force=-40.0)
                p.stepSimulation()
            freed = p.getJointState(arm, index)[0]
            self.assertLess(freed, held - math.radians(0.1))
        finally:
            p.disconnect(connection)


if __name__ == "__main__":
    unittest.main()
