"""Actual ARM FreeRTOS C APF/IK -> UART proposal -> Bullet all-link gate
-> separate ARM C trajectory/PID command -> Bullet feedback. SOFTWARE ONLY.

This is deliberately NOT the host C APF bridge: the planner's decision
is observed exclusively in a new firmware UART proposal response. A status
ACK is never counted as physical movement. Full-link clearance is checked
before any execution request and on every physical simulation step.
"""
from __future__ import annotations
import argparse
import json
import math
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT / "simulation" / "pybullet_ur5" / "src"
sys.path.insert(0, str(SIM))

import qemu_uart_receiver as wire  # noqa: E402
from stage5_qemu_uart_closed_loop import (  # noqa: E402
    frame, FrameStream, vacant_tcp_port, join_qemu_serial,
    FEEDBACK_PERIOD_S, PHYSICS_DT_S,
)
from stage4_dynamic_closed_loop import (  # noqa: E402
    URDF, START_RAW, joint_indices, angles_raw,
)
from stage5_avoidance_uart_gate import load_scene  # noqa: E402
from stage5_collision_validation import validate_joint_path  # noqa: E402

def run(elf: Path, output_dir: Path, qemu_bin: str, timeout_s: float) -> dict:
    import pybullet as p
    scene, policy = load_scene("blocked")
    box = scene["obstacle"]
    offsets = [int(v) for v in scene["goal_offset_mm"]]
    center_mm = [int(v) for v in box["center_offset_ee_mm"]]
    half_mm = [int(round(1000.0*v)) for v in box["half_extents_m"]]
    if any(abs(v-x)>1e-7 for v,x in zip(
            box["half_extents_m"], (n/1000 for n in half_mm))):
        raise ValueError("test rectangle edges not exactly representable in mm")
    bounds_min = [a-b for a,b in zip(center_mm,half_mm)]
    bounds_max = [a+b for a,b in zip(center_mm,half_mm)]
    apf = scene["apf"]
    apf_fields = ("attraction_gain", "repulsion_gain",
                  "influence_distance_mm", "clearance_mm",
                  "maximum_step_mm", "goal_tolerance_mm")
    proposal_payload = struct.pack(
        "<B9h6f", 51, *offsets, *bounds_min, *bounds_max,
        *(float(apf[name]) for name in apf_fields))
    assert len(proposal_payload)==43
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir/"result.json").unlink(missing_ok=True)
    port = vacant_tcp_port()
    qemu = subprocess.Popen([
        qemu_bin, "-M", "mps2-an386", "-kernel", str(elf.resolve()),
        "-display", "none", "-monitor", "none",
        "-serial", f"tcp:127.0.0.1:{port},server,nowait",
    ], stdout=subprocess.DEVNULL, stderr=(output_dir/"qemu_stderr.log").open("w"))
    client = -1
    result = {
        "passed": False, "scope": "ARM_FreeRTOS_C_APF_IK_UART_Bullet_full_link",
        "new_apf_proposals": 0, "proposal_status": None,
        "joint_motion_acks": 0, "physical_settled_frames": 0,
        "full_link_safe": False, "joint_motion_submitted_only_after_gate": False,
        "uart_checksum_errors": 0,
    }
    try:
        with join_qemu_serial(port,qemu) as sock:
            client=p.connect(p.DIRECT)
            if client<0: raise RuntimeError("PyBullet unavailable")
            p.setGravity(0,0,-9.81,physicsClientId=client)
            p.setTimeStep(PHYSICS_DT_S,physicsClientId=client)
            arm=p.loadURDF(str(URDF),useFixedBase=True,physicsClientId=client)
            joints=joint_indices(p,arm)
            for index,raw in zip(joints,START_RAW):
                p.resetJointState(arm,index,math.radians(raw/100),
                                  physicsClientId=client)
            efforts=[float(p.getJointInfo(arm,j,physicsClientId=client)[10])
                     for j in joints]
            p.setJointMotorControlArray(
                arm,joints,p.POSITION_CONTROL,
                targetPositions=[math.radians(v/100) for v in START_RAW],
                forces=efforts, physicsClientId=client)
            for _ in range(100): p.stepSimulation(physicsClientId=client)
            start=tuple(angles_raw(p,arm,joints))
            ee=p.getLinkState(arm,p.getNumJoints(arm,physicsClientId=client)-1,
                              computeForwardKinematics=True,
                              physicsClientId=client)[4]
            box_center=[ee[i]+center_mm[i]/1000 for i in range(3)]
            shape=p.createCollisionShape(
                p.GEOM_BOX,halfExtents=box["half_extents_m"],
                physicsClientId=client)
            obstacle=p.createMultiBody(
                baseMass=0,baseCollisionShapeIndex=shape,
                basePosition=box_center,physicsClientId=client)
            parser=FrameStream()
            feedback_acks=0
            proposal_sent=False
            move_sent=False
            move_ack=None
            candidate=None
            settle=0
            max_change=0.0
            frames=0
            last_feedback=start
            # Startup transaction: the ARM firmware must ACK real feedback
            # before it may read FK and issue its on-device proposal.
            sock.sendall(frame(
                wire.CMD_JOINT_STATE,struct.pack("<6h",*last_feedback)))
            deadline=time.monotonic()+timeout_s
            while time.monotonic()<deadline:
                if qemu.poll() is not None:
                    raise RuntimeError("ARM QEMU terminated before target settle")
                try: chunk=sock.recv(8192)
                except BlockingIOError: chunk=None
                if chunk==b"": raise RuntimeError("ARM UART disconnected")
                if not chunk:
                    time.sleep(0.001)
                    continue
                for command,payload in parser.consume(chunk):
                    if command==wire.CMD_JOINT_STATE_ACK and len(payload)==12:
                        feedback_acks+=1
                        if feedback_acks>=3 and not proposal_sent:
                            # This is the ONLY source of the APF/IK waypoint.
                            sock.sendall(frame(
                                wire.CMD_REQUEST_APF_JOINT_PROPOSAL,
                                proposal_payload))
                            proposal_sent=True
                    elif (command==wire.CMD_APF_JOINT_PROPOSAL_RESPONSE and
                          len(payload)==27):
                        seq,status,*values=struct.unpack("<Bb3f6hB",payload)
                        if seq!=51 or status!=0:
                            raise RuntimeError(
                                f"ARM C APF/IK rejected proposal {seq=} {status=}")
                        if result["new_apf_proposals"]!=0:
                            raise RuntimeError("Unexpected duplicate C APF response")
                        xyz=values[:3]
                        candidate=tuple(int(x) for x in values[3:9])
                        reached=int(values[9])
                        if reached:
                            raise RuntimeError("APF falsely declared blocked target reached")
                        current=tuple(angles_raw(p,arm,joints))
                        # PyBullet knows the exact robot link shapes. The ARM
                        # APF is end-effector-only; it may NEVER self-authorize
                        # a movement past an unvalidated elbow/wrist/forearm.
                        safe=validate_joint_path(
                            p,arm,[obstacle],joints,
                            [[math.radians(x/100) for x in current],
                             [math.radians(x/100) for x in candidate]],
                            maximum_joint_step_rad=policy["max_joint_step_rad"],
                            clearance_m=scene["minimum_full_link_clearance_mm"]/1000,
                            maximum_samples=policy["max_collision_samples"],
                            physics_client_id=client)
                        result.update(
                            new_apf_proposals=1, proposal_status=status,
                            proposed_joint_raw=list(candidate),
                            proposed_cartesian_mm=list(xyz),
                            proposal_goal_reached=bool(reached),
                            full_link_gate=safe,
                        )
                        if not safe["passed"]:
                            raise RuntimeError("ARM proposed path fails full-link gate")
                        if move_sent:
                            raise RuntimeError("Redundant execution request")
                        result["joint_motion_submitted_only_after_gate"]=True
                        sock.sendall(frame(
                            wire.CMD_REQUEST_JOINT_MOVE,
                            bytes([52])+struct.pack("<6h",*candidate)))
                        move_sent=True
                    elif command==wire.CMD_JOINT_MOVE_ACK and len(payload)==2:
                        seq,status=struct.unpack("<Bb",payload)
                        if seq!=52 or status!=0:
                            raise RuntimeError(
                                f"ARM rejected full-link-approved waypoint: {seq=} {status=}")
                        result["joint_motion_acks"]+=1
                        move_ack=status
                    elif command==wire.CMD_SET_JOINT_TARGETS and len(payload)==12:
                        values=struct.unpack("<6h",payload)
                        frames+=1
                        if move_ack==0:
                            p.setJointMotorControlArray(
                                arm,joints,p.POSITION_CONTROL,
                                targetPositions=[math.radians(v/100)
                                                 for v in values],
                                forces=efforts,physicsClientId=client)
                        for _ in range(round(FEEDBACK_PERIOD_S/PHYSICS_DT_S)):
                            p.stepSimulation(physicsClientId=client)
                        observed=tuple(angles_raw(p,arm,joints))
                        last_feedback=observed
                        sock.sendall(frame(
                            wire.CMD_JOINT_STATE,struct.pack("<6h",*observed)))
                        max_change=max(
                            max_change,max(abs(a-b)/100
                                           for a,b in zip(observed,start)))
                        if candidate is not None and move_ack==0:
                            # Any real collision even between preflight samples
                            # immediately kills the test; no fabricated PASS.
                            distances=p.getClosestPoints(
                                arm,obstacle,
                                scene["minimum_full_link_clearance_mm"]/1000,
                                physicsClientId=client)
                            if any(c[8] < scene["minimum_full_link_clearance_mm"]/1000
                                   for c in distances):
                                raise RuntimeError("Runtime full-link collision")
                            err=max(abs(a-b)/100
                                    for a,b in zip(observed,candidate))
                            settle=settle+1 if err<=0.15 else 0
                            if settle>=15 and max_change>=0.20:
                                result.update(
                                    passed=True,physical_settled_frames=settle,
                                    max_motion_deg=max_change,
                                    final_error_deg=err,full_link_safe=True,
                                    uart_checksum_errors=parser.bad_checksum,
                                    received_target_frames=frames,
                                    feedback_acks=feedback_acks,
                                )
                                break
                if result["passed"]: break
            if not result["passed"]:
                raise TimeoutError(
                    f"ARM C APF/IK UART physics did not settle: "
                    f"{result['new_apf_proposals']=}, {move_ack=}, "
                    f"{settle=}, {frames=}")
            if parser.bad_checksum:
                raise RuntimeError("UART framing checksum error")
    except Exception as error:
        result["passed"]=False
        result["error"]=repr(error)
        raise
    finally:
        if client>=0: p.disconnect(physicsClientId=client)
        if qemu.poll() is None: qemu.terminate()
        try: qemu.wait(timeout=3)
        except subprocess.TimeoutExpired: qemu.kill()
        (output_dir/"result.json").write_text(
            json.dumps(result,indent=2),encoding="utf-8")
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--elf",required=True,type=Path)
    parser.add_argument("--output-dir",required=True,type=Path)
    parser.add_argument("--qemu",default="qemu-system-arm")
    parser.add_argument("--timeout-s",type=float,default=105.0)
    args=parser.parse_args()
    if not args.elf.is_file(): parser.error("ELF missing")
    result=run(args.elf,args.output_dir,args.qemu,args.timeout_s)
    print(json.dumps(result,indent=2))
    if not result["passed"]: raise SystemExit(1)


if __name__=="__main__":
    main()
