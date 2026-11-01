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

def describe_joint_tracking(candidate_raw, observed_raw, commanded_raw) -> dict:
    """Read-only centidegree diagnostics; never used for safety decisions."""
    if any(len(seq) != wire.JOINT_COUNT for seq in
           (candidate_raw, observed_raw, commanded_raw)):
        raise ValueError("tracking diagnostics require six joint samples")
    scale = wire.JOINT_ANGLE_SCALE
    return {
        "candidate_target_raw": list(candidate_raw),
        "last_physical_feedback_raw": list(observed_raw),
        "last_firmware_command_raw": list(commanded_raw),
        "feedback_minus_candidate_deg": [
            round((actual - target) / scale, 4)
            for actual, target in zip(observed_raw, candidate_raw)
        ],
        "feedback_minus_command_deg": [
            round((actual - command) / scale, 4)
            for actual, command in zip(observed_raw, commanded_raw)
        ],
        "firmware_command_minus_candidate_deg": [
            round((command - target) / scale, 4)
            for command, target in zip(commanded_raw, candidate_raw)
        ],
    }


def decode_firmware_motion_diagnostic(metric_name: str, payload: bytes) -> int:
    """Decode an existing UART diagnostic response, with signed fault code."""
    if len(payload) != 4:
        raise ValueError("firmware diagnostic response must be 4 bytes")
    value = struct.unpack("<I", payload)[0]
    if (metric_name == "last_fault" or metric_name.startswith((
            "trace_reference_raw_j",
            "trace_feedback_raw_j",
            "trace_pid_correction_millideg_j",
            "trace_limiter_position_millicentideg_j",
            "trace_command_raw_j",
        ))) and value >= (1 << 31):
        value -= 1 << 32
    return value


# Existing firmware diagnostics selector registry, not a duplicated wire map.
MOTION_DIAGNOSTIC_SELECTORS = tuple(
    (name, wire.SIM_MOTION_DIAGNOSTIC_SELECTORS[name])
    for name in (
        "requests_accepted",
        "trajectory_samples",
        "controller_steps",
        "controller_faults",
        "planner_faults",
        "feedback_stale_count",
        "last_fault",
        "monitoring_ticks",
    )
) + tuple(
    (name, selector)
    for name, selector in wire.SIM_MOTION_DIAGNOSTIC_SELECTORS.items()
    if name.startswith("trace_")
)


def summarize_stalled_control_trace(metrics: dict) -> dict | None:
    """Group six-axis QEMU-only trace selectors; never alter motion outcome."""
    groups=(
        "reference_raw", "feedback_raw", "pid_correction_millideg",
        "limiter_position_millicentideg", "command_raw",
    )
    scalars=(
        "rtos_tick", "control_dt_us", "reference_age_ticks",
        "request_generation",
    )
    keys=[f"trace_{name}" for name in scalars]
    keys += [
        f"trace_{name}_j{axis}"
        for name in groups for axis in range(1, wire.JOINT_COUNT+1)
    ]
    if any(name not in metrics for name in keys):
        return None
    result={name:metrics[f"trace_{name}"] for name in scalars}
    result.update({
        name:[metrics[f"trace_{name}_j{axis}"]
              for axis in range(1, wire.JOINT_COUNT+1)]
        for name in groups
    })
    # The selector replies are acquired sequentially over UART, not at a
    # single instant. Only interpret them for a verified stalled command.
    result["coherent_across_axes"]=False
    return result


def require_runtime_full_link_clearance(
    bullet, robot: int, obstacle: int, clearance_m: float, client: int,
) -> int:
    """Check the BASE and EVERY URDF link at the actual simulated pose.

    PyBullet's default linkIndexA=-1 checks the base only. A single
    getClosestPoints(robot, obstacle, ...) call must never be reported as
    whole-arm safety evidence. Returns the number of individual link queries.
    This is still a discrete 10 ms runtime gate, not swept-volume proof.
    """
    if not math.isfinite(clearance_m) or clearance_m <= 0.0:
        raise ValueError("runtime full-link clearance must be finite and positive")
    count = bullet.getNumJoints(robot, physicsClientId=client)
    if type(count) is not int or count < 1:
        raise ValueError("invalid robot joint count in runtime clearance gate")
    for link in range(-1, count):
        contacts = bullet.getClosestPoints(
            robot, obstacle, clearance_m, linkIndexA=link, linkIndexB=-1,
            physicsClientId=client,
        )
        for contact in contacts:
            if len(contact) <= 8:
                raise RuntimeError(f"Malformed runtime contact for link={link}")
            distance = float(contact[8])
            if not math.isfinite(distance) or distance <= clearance_m:
                raise RuntimeError(
                    f"Runtime full-link clearance violation: link={link}, "
                    f"distance_m={distance}, required_m={clearance_m}"
                )
    return count + 1


def run(elf: Path, output_dir: Path, qemu_bin: str, timeout_s: float,
        *, required_proposals: int = 1,
        full_route: bool = False) -> dict:
    if not isinstance(required_proposals, int) or not 1 <= required_proposals <= 6:
        raise ValueError("on-firmware APF integration step count must be 1..6")
    if full_route and required_proposals != 1:
        raise ValueError("full-route mode controls length from actual APF goal completion")
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
    # Each request carries an offset relative to the *current* physical EE;
    # keep both the static world obstacle and absolute Cartesian goal fixed.
    # The host does not calculate APF forces, C IK branches or trajectories.
    def apf_payload(seq: int, current_ee, absolute_goal, obstacle_center):
        goal_delta=[round((absolute_goal[i]-current_ee[i])*1000)
                    for i in range(3)]
        lo=[round((obstacle_center[i]-box["half_extents_m"][i]-
                   current_ee[i])*1000) for i in range(3)]
        hi=[round((obstacle_center[i]+box["half_extents_m"][i]-
                   current_ee[i])*1000) for i in range(3)]
        values=(*goal_delta,*lo,*hi)
        if any(not -32768 <= v <= 32767 for v in values):
            raise ValueError("obstacle/goal offsets exceed explicit UART i16 mm range")
        return struct.pack(
            "<B9h6f", seq, *values,
            *(float(apf[name]) for name in apf_fields))
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
        "new_apf_proposals": 0, "required_apf_proposals": required_proposals,
        "full_route_mode": full_route, "route_goal_index": 0,
        "route_complete": False, "proposal_status": None, "approved_legs": [],
        "joint_motion_acks": 0, "physical_settled_frames": 0,
        "full_link_safe": False, "joint_motion_submitted_only_after_gate": False,
        "runtime_full_link_queries": 0,
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
            route_goals=[
                [ee[i]+float(delta[i])/1000 for i in range(3)]
                for delta in (
                    list(scene.get("via_offsets_ee_mm",[]))+
                    [scene["goal_offset_mm"]])
            ] if full_route else [
                [ee[i]+offsets[i]/1000 for i in range(3)]
            ]
            goal_index=0
            absolute_goal=route_goals[goal_index]
            shape=p.createCollisionShape(
                p.GEOM_BOX,halfExtents=box["half_extents_m"],
                physicsClientId=client)
            obstacle=p.createMultiBody(
                baseMass=0,baseCollisionShapeIndex=shape,
                basePosition=box_center,physicsClientId=client)
            parser=FrameStream()
            feedback_acks=0
            proposal_sent=False
            proposal_seq=51
            motion_seq=52
            move_sent=False
            move_ack=None
            candidate=None
            settle=0
            max_change=0.0
            frames=0
            previous_firmware_command=None
            unchanged_firmware_command_frames=0
            diagnostic_waiting=None
            diagnostic_pending=[]
            diagnostic_snapshot=None
            diagnostic_leg=-1
            diagnostic_snapshot_index=0
            current_motion_start_frame=0
            best_motion_error_deg=math.inf
            within_tolerance_frames=0
            last_progress_report=time.monotonic()
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
                            current_ee=p.getLinkState(
                                arm,p.getNumJoints(arm,physicsClientId=client)-1,
                                computeForwardKinematics=True,
                                physicsClientId=client)[4]
                            sock.sendall(frame(
                                wire.CMD_REQUEST_APF_JOINT_PROPOSAL,
                                apf_payload(proposal_seq,current_ee,
                                            absolute_goal,box_center)))
                            proposal_sent=True
                    elif (command==wire.CMD_APF_JOINT_PROPOSAL_RESPONSE and
                          len(payload)==27):
                        seq,status,*values=struct.unpack("<Bb3f6hB",payload)
                        if seq!=proposal_seq or status!=0:
                            raise RuntimeError(
                                f"ARM C APF/IK rejected proposal {seq=} {status=}")
                        if move_sent:
                            raise RuntimeError("Unexpected APF proposal while motion active")
                        xyz=values[:3]
                        candidate=tuple(int(x) for x in values[3:9])
                        reached=int(values[9])
                        if reached:
                            if not full_route:
                                raise RuntimeError("APF falsely declared blocked target reached")
                            current_ee=p.getLinkState(
                                arm,p.getNumJoints(arm,physicsClientId=client)-1,
                                computeForwardKinematics=True,
                                physicsClientId=client)[4]
                            distance_mm=1000*math.dist(current_ee,absolute_goal)
                            if distance_mm>apf["goal_tolerance_mm"]+1.0:
                                raise RuntimeError(
                                    f"ARM reports reached but physical EE error {distance_mm:.3f}mm")
                            if goal_index+1==len(route_goals):
                                if not result["approved_legs"]:
                                    raise RuntimeError("No physical C motions in route")
                                result.update(
                                    passed=True,route_complete=True,
                                    final_cartesian_error_mm=distance_mm,
                                    physical_settled_frames=15,
                                    max_motion_deg=max_change,
                                    final_error_deg=result["approved_legs"][-1]["final_error_deg"],
                                    full_link_safe=True,
                                    uart_checksum_errors=parser.bad_checksum,
                                    received_target_frames=frames,
                                    feedback_acks=feedback_acks,
                                )
                                break
                            goal_index+=1
                            result["route_goal_index"]=goal_index
                            absolute_goal=route_goals[goal_index]
                            proposal_seq=(proposal_seq+2)&255
                            motion_seq=(motion_seq+2)&255
                            move_sent=False
                            move_ack=None
                            candidate=None
                            settle=0
                            sock.sendall(frame(
                                wire.CMD_REQUEST_APF_JOINT_PROPOSAL,
                                apf_payload(proposal_seq,current_ee,
                                            absolute_goal,box_center)))
                            continue
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
                            new_apf_proposals=result["new_apf_proposals"]+1,
                            proposal_status=status,
                            proposed_joint_raw=list(candidate),
                            proposed_cartesian_mm=list(xyz),
                            proposal_goal_reached=bool(reached),
                            full_link_gate=safe,
                        )
                        if full_route and result["new_apf_proposals"] > (
                                len(route_goals) * int(scene["waypoint_capacity"])):
                            raise RuntimeError("ARM APF route exceeded scenario waypoint budget")
                        result["approved_legs"].append({
                            "proposal_seq":proposal_seq,
                            "motion_seq":motion_seq,
                            "target_raw":list(candidate),
                            "full_link_gate":safe,
                            "ack":False,"settled":False,
                        })
                        if not safe["passed"]:
                            raise RuntimeError("ARM proposed path fails full-link gate")
                        if move_sent:
                            raise RuntimeError("Redundant execution request")
                        result["joint_motion_submitted_only_after_gate"]=True
                        sock.sendall(frame(
                            wire.CMD_REQUEST_JOINT_MOVE,
                            bytes([motion_seq])+struct.pack("<6h",*candidate)))
                        move_sent=True
                    elif command==wire.CMD_DIAGNOSTICS_RESPONSE and len(payload)==4:
                        if diagnostic_waiting is not None and diagnostic_snapshot is not None:
                            name, _selector=diagnostic_waiting
                            diagnostic_snapshot["metrics"][name] = (
                                decode_firmware_motion_diagnostic(name, payload))
                            diagnostic_waiting=None
                            if not diagnostic_pending:
                                snapshot_trace=summarize_stalled_control_trace(
                                    diagnostic_snapshot["metrics"])
                                if snapshot_trace is not None:
                                    diagnostic_snapshot["control_trace"]=snapshot_trace
                    elif command==wire.CMD_JOINT_MOVE_ACK and len(payload)==2:
                        seq,status=struct.unpack("<Bb",payload)
                        if seq!=motion_seq or status!=0:
                            raise RuntimeError(
                                f"ARM rejected full-link-approved waypoint: {seq=} {status=}")
                        result["joint_motion_acks"]+=1
                        result["approved_legs"][-1]["ack"]=True
                        move_ack=status
                        current_motion_start_frame=frames
                        best_motion_error_deg=math.inf
                        within_tolerance_frames=0
                    elif command==wire.CMD_SET_JOINT_TARGETS and len(payload)==12:
                        values=struct.unpack("<6h",payload)
                        frames+=1
                        if values==previous_firmware_command:
                            unchanged_firmware_command_frames+=1
                        else:
                            unchanged_firmware_command_frames=0
                        previous_firmware_command=values
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
                            result["runtime_full_link_queries"] += (
                                require_runtime_full_link_clearance(
                                    p, arm, obstacle,
                                    scene["minimum_full_link_clearance_mm"]/1000,
                                    client,
                                )
                            )
                            err=max(abs(a-b)/100
                                    for a,b in zip(observed,candidate))
                            settle=settle+1 if err<=0.15 else 0
                            best_motion_error_deg=min(best_motion_error_deg, err)
                            if err<=0.15:
                                within_tolerance_frames+=1
                            diagnostics=describe_joint_tracking(
                                candidate, observed, values)
                            diagnostics.update(
                                unchanged_firmware_command_frames=unchanged_firmware_command_frames,
                                leg_number=len(result["approved_legs"]),
                                frames_since_ack=frames-current_motion_start_frame,
                                last_max_error_deg=round(err,4),
                                best_max_error_deg=round(best_motion_error_deg,4),
                                within_tolerance_frames=within_tolerance_frames,
                                consecutive_settle_frames=settle,
                            )
                            result["last_motion_diagnostics"]=diagnostics
                            current_leg=len(result["approved_legs"])
                            # A stalled command must not silently masquerade
                            # as a long move. Request existing on-ARM counters
                            # through the read-only diagnostics UART opcode.
                            # Capture twice so task activity can be compared.
                            if (err>0.15 and
                                    unchanged_firmware_command_frames>=200 and
                                    not diagnostic_pending and
                                    diagnostic_waiting is None and
                                    (diagnostic_leg!=current_leg or
                                     (diagnostic_snapshot_index==1 and
                                      unchanged_firmware_command_frames>=700))):
                                if diagnostic_leg!=current_leg:
                                    diagnostic_leg=current_leg
                                    diagnostic_snapshot_index=0
                                diagnostic_snapshot_index+=1
                                diagnostic_snapshot={
                                    "leg_number":current_leg,
                                    "frame":frames,
                                    "snapshot_index":diagnostic_snapshot_index,
                                    "metrics":{},
                                }
                                result.setdefault(
                                    "firmware_runtime_diagnostics",[]
                                ).append(diagnostic_snapshot)
                                diagnostic_pending=list(MOTION_DIAGNOSTIC_SELECTORS)
                            if diagnostic_waiting is None and diagnostic_pending:
                                diagnostic_waiting=diagnostic_pending.pop(0)
                                sock.sendall(frame(
                                    wire.CMD_GET_DIAGNOSTICS,
                                    bytes([diagnostic_waiting[1]])))
                            now=time.monotonic()
                            if now-last_progress_report>=15.0:
                                print(
                                    "[Stage5 APF] leg="
                                    f'{len(result["approved_legs"])} '
                                    f'frame={frames} err={err:.3f}deg '
                                    f'best={best_motion_error_deg:.3f}deg '
                                    f'settle={settle}/15 '
                                    f'observed={list(observed)} '
                                    f'command={list(values)}',
                                    flush=True,
                                )
                                last_progress_report=now
                            if settle>=15 and max_change>=0.20:
                                result["approved_legs"][-1]["settled"]=True
                                result["approved_legs"][-1]["final_error_deg"]=err
                                if not full_route and result["new_apf_proposals"]>=required_proposals:
                                    result.update(
                                        passed=True,physical_settled_frames=settle,
                                        max_motion_deg=max_change,
                                        final_error_deg=err,full_link_safe=True,
                                        uart_checksum_errors=parser.bad_checksum,
                                        received_target_frames=frames,
                                        feedback_acks=feedback_acks,
                                    )
                                    break
                                # This is the only transition to the next
                                # on-ARM C APF/IK step. Re-observe the actual
                                # new EE and ask the same embedded planner
                                # with re-based goal/box offsets. Never issue
                                # a new move before physical settlement.
                                proposal_seq=(proposal_seq+2)&255
                                motion_seq=(motion_seq+2)&255
                                proposal_sent=True
                                move_sent=False
                                move_ack=None
                                candidate=None
                                settle=0
                                current_ee=p.getLinkState(
                                    arm,p.getNumJoints(
                                        arm,physicsClientId=client)-1,
                                    computeForwardKinematics=True,
                                    physicsClientId=client)[4]
                                sock.sendall(frame(
                                    wire.CMD_REQUEST_APF_JOINT_PROPOSAL,
                                    apf_payload(proposal_seq,current_ee,
                                                absolute_goal,box_center)))
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
        if not result["passed"] and result.get("firmware_runtime_diagnostics"):
            snapshots=result["firmware_runtime_diagnostics"]
            print("[Stage5] ARM control trace on failure:",
                  json.dumps(snapshots,ensure_ascii=False),flush=True)
        (output_dir/"result.json").write_text(
            json.dumps(result,indent=2),encoding="utf-8")
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--elf",required=True,type=Path)
    parser.add_argument("--output-dir",required=True,type=Path)
    parser.add_argument("--qemu",default="qemu-system-arm")
    parser.add_argument("--timeout-s",type=float,default=105.0)
    parser.add_argument("--proposals",type=int,default=1,
                        help="number of repeated firmware C APF physical steps")
    parser.add_argument("--full-route",action="store_true",
                        help="iteratively run the entire scene via/goal path via real ARM APF")
    args=parser.parse_args()
    if not args.elf.is_file(): parser.error("ELF missing")
    result=run(args.elf,args.output_dir,args.qemu,args.timeout_s,
               required_proposals=args.proposals,full_route=args.full_route)
    print(json.dumps(result,indent=2))
    if not result["passed"]: raise SystemExit(1)


if __name__=="__main__":
    main()
