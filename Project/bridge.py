import numpy as np
from pathlib import Path
from AI_module import LegoTampanda, convert_workbenchmark_yaml, move_linear, move_vertical, get_brick_and_hand_pose, place_at, get_place_target, get_stack_target
from tampanda.planners.grasp_planner import GraspType
from tampanda import DomainBridge, GraspPlanner
from parser import get_brick_names, block_rename
from mj_bridge.gym_env import LegoBenchEnv 
from scipy.spatial.transform import Rotation 

DOMAIN_PATH = Path(__file__).resolve().with_name("domain.pddl")

GRASP_CONFIGS = {}
APPROACH_OFFSET = np.array([0, 0, 0.15])
LIFT_HEIGHT = 0.20
BLOCKS = []
HOME_Q = None 

# Height of the hand body origin above the brick body origin while grasping.
#
# LegoIK controls the MuJoCo body "hand". tampanda's GraspPlanner, however,
# returns poses of tampanda's *attachment_site*, which lies 0.09 m further down
# the tool axis. Feeding those positions to LegoIK commanded the hand about 6 cm
# below the table: the arm pressed its fingertips onto the table and gripped the
# brick at its very bottom. That works while the support has the same footprint
# (Tier 1), but in an offset stack (Tier 2) the fingertips hit the exposed studs
# of the support, the brick stays 4 mm above its seat and the arm levers the
# whole tower off the plate.
#
# 0.1034 m is the grasp height of lego_sim's own reference executor
# (gripper_offset_m - (grasp_descent_m - approach_height_m) in executor.yaml).
# It keeps the fingertips ~9 mm above the brick's bottom face, clear of the
# 4 mm studs. Only the orientation of a GraspPlanner candidate is used.
HAND_ABOVE_BRICK = 0.1034

# Half extents (block-local x, y, z) from lego_sim's part registry.
BRICK_HALF_SIZE = {
    "4x2": np.array([0.032, 0.016, 0.0095]),
    "2x2": np.array([0.016, 0.016, 0.0095]),
}

# Each finger closes this far past the brick surface (position-controlled grip).
GRIP_SQUEEZE = 0.002
GRIPPER_OPEN = 0.04
# Simulation steps (0.04 s each) over which the fingers open.
RELEASE_STEPS = 25

# lego_sim's direct Python API has no stud clutch: a released brick is held by
# gravity and contact only. STABILITY_BIAS is the lateral shift that moves the
# centre of mass of a half-supported brick back over its support, see
# stability_bias(). Studs leave 2.8 mm of play and the benchmark tolerates 6 mm.
STABILITY_BIAS = 0.0025
STABILITY_MARGIN = 0.001

# Overtravel below the seat when a brick is stacked. Zero: a brick is lowered
# onto its seat and not pressed. lego_sim has no interference fit between studs
# and cavity, so pressing gains nothing, and the stiff position-controlled arm
# answers 1 mm of overtravel with about 40 N. On a half-supported brick that
# force acts outside the studs the brick rests on: it tilts the brick into the
# edge of its support, and the contact solver may eject it (we recorded contact
# forces above 30 kN). Whether that happened depended on the initial layout of
# the bricks: 19 of 20 tasks succeeded with seed 42, 15 of 20 with seed 0.
# If a press is configured, place_at() takes it off again before releasing.
PRESS_DEPTH = 0.0

# A placement counts as failed when the released brick ends up further than this
# from its benchmark target (it fell or tipped). Deliberately looser than the
# benchmark tolerance, which is applied to the whole assembly at the end.
PLACED_XY_TOL = 0.010
PLACED_Z_TOL = 0.006

held_relative_pos = {}
held_relative_rot = {}

env = None 
name_mapping = {}

tampanda = None 
grasp_planner = None 
bridge = None 


"""
Grasp feasibility checking as implemented in the exercise sheets.
"""
CLEAR, ALIGN = 0.10, 0.045 # finger-corridot length / lateral tolerance (m)
def finger_axis(candidate):
    """
    World XY direction along which the fingers close (y axis of the hand frame).
    GraspPlanner aligns top-down grasps with the block, so for a block that is
    yawed by 90 deg TOP_DOWN_X closes along world Y, not world X.
    """
    w, x, y, z = candidate.grasp_quat
    axis = np.array([2 * (x * y - w * z), 1 - 2 * (x * x + z * z)])
    return axis / np.linalg.norm(axis)

def corridor_blocked(env, other, block, candidate):
    """
    True if "other" sits in "block"'s gripper corridor for this top-down grasp:
    closer than CLEAR along the finger axis and closer than ALIGN across it.
    """
    env_other = name_mapping[other]
    env_block = name_mapping[block]

    offset = (env.data.body(env_other).xpos - env.data.body(env_block).xpos)[:2]
    axis = finger_axis(candidate)

    along = abs(offset @ axis)
    across = abs(offset[0] * axis[1] - offset[1] * axis[0])

    return bool(along < CLEAR and across < ALIGN)

def eval_ik_feasible(env, fluents, config: str) -> bool:
    """
    True iff no other block is in this config's grasp corridor.
    """
    block = GRASP_CONFIGS[config]["block"]
    candidate = GRASP_CONFIGS[config]["candidate"]

    for entry in BLOCKS:
        if entry != block:
            if corridor_blocked(env, entry, block, candidate):
                return False 
    return True 

def grasp_width(item, candidate):
    """
    Extent of the brick along the finger axis (block-local X or Y).
    """
    half = BRICK_HALF_SIZE[item[-3:]]
    return 2.0 * (half[0] if candidate.grasp_type == GraspType.TOP_DOWN_X else half[1])

def feasible_config(env, item):
    """ 
    A grasp candidate for "item" that is collision-free in the current scene (or None).
    Candidates that close across the narrow side of the brick come first: a 4x2
    brick is 64 mm long, which leaves the 80 mm gripper only 8 mm per finger to
    open, while across its 32 mm side the fingers clear the brick by 24 mm.
    """
    configs = [c for c in GRASP_CONFIGS if GRASP_CONFIGS[c]["block"] == item]
    configs.sort(key=lambda c: grasp_width(item, GRASP_CONFIGS[c]["candidate"]))

    for c in configs:
        if eval_ik_feasible(env, {}, c):
            return GRASP_CONFIGS[c]["candidate"]
    return None


def generate_grasps(env, blocks, grasp_planner, name_mapping):
    for block in blocks: 
        env_name = name_mapping[block]

        pos = env.data.body(env_name).xpos.copy()
        quat = env.data.body(env_name).xquat.copy()

        candidates = grasp_planner.generate_candidates(pos, BRICK_HALF_SIZE[block[-3:]], quat)
        i = 0
        for candidate in candidates:
            GRASP_CONFIGS[f"cfg_{block}_{i}"] = {"block": block, "candidate": candidate}
            i += 1 


def flip_about_approach_axis(quat):
    """
    The same grasp with the hand turned by 180 deg about its approach (z) axis.
    A parallel gripper is symmetric under this turn, so both orientations hold
    the brick identically; one of them is usually easier to reach.
    """
    w, x, y, z = quat
    return np.array([-z, y, -x, w])


"""
Functions used to open/close gripper.
"""
def close_gripper(env, width):
    action = env.data.ctrl.copy()
    action[7:9] = max(0.0, width / 2.0 - GRIP_SQUEEZE)
    for _ in range(10):
        env.step(action)
def open_gripper(env):
    """
    Opens the fingers along a ramp instead of at once. A brick that turned
    slightly inside the grasp is wedged between the fingers; letting go
    abruptly releases that tension as a twist and spins the brick on its studs,
    which leave it about 10 deg of rotational play.
    """
    action = env.data.ctrl.copy()
    start = env.data.qpos[7:9].copy()

    for fraction in np.linspace(0.0, 1.0, RELEASE_STEPS + 1)[1:]:
        action[7:9] = start + fraction * (GRIPPER_OPEN - start)
        env.step(action)

    for _ in range(10):
        env.step(action)


"""
Creates the name mapping for YAML, PDDL, LegoSim etc.
"""
def generate_name_mapping(env, yaml_path):
    pddl_names = get_brick_names(yaml_path, False)
    pddl_names = pddl_names.split()

    env_names = []

    for i in range(env.model.nbody):
        env_names.append(env.model.body(i).name)

    name_mapping = {}

    for name in pddl_names:
        name_mapping[block_rename(name)] = next(
            (env_name for env_name in env_names if name in env_name),
            None
        )

    return name_mapping


"""
Geometry of the target assembly (from the benchmark manifest).
"""
def benchmark_target(env, item_pddl):
    """
    Target entry of the episode manifest for a PDDL brick name.
    """
    parts = item_pddl.split("_")
    benchmark_id = "_".join(parts[2:] + parts[:2])

    return next(
        (t for t in env.episode_manifest["target_blocks"] if t["id"] == benchmark_id),
        None
    )

def footprint(target):
    """
    Axis-aligned (x_min, x_max, y_min, y_max) of a brick at its target pose.
    """
    half = BRICK_HALF_SIZE[target["type"][-3:]]
    quarter_turns = int(round(target["yaw_rad"] / (np.pi / 2)))
    hx, hy = (half[1], half[0]) if quarter_turns % 2 else (half[0], half[1])
    x, y = target["position"][0], target["position"][1]

    return x - hx, x + hx, y - hy, y + hy

def stability_bias(env, item_pddl, support_pddl):
    """
    Shift that moves a brick's centre of mass safely over its support.

    A brick that overlaps its support by only half has its centre exactly above
    the edge of the shared area. Real LEGO holds it by the clutch of the studs;
    lego_sim's Python API does not model that, so after release the smallest
    placement error towards the free side tips the brick over. Such a brick is
    therefore placed STABILITY_BIAS towards the support.
    """
    item_target = benchmark_target(env, item_pddl)
    item, support = footprint(item_target), footprint(benchmark_target(env, support_pddl))
    centre = item_target["position"]

    shift = np.zeros(3)
    for axis in (0, 1):
        shared_min = max(item[2 * axis], support[2 * axis])
        shared_max = min(item[2 * axis + 1], support[2 * axis + 1])

        if centre[axis] > shared_max - STABILITY_MARGIN:
            shift[axis] = -STABILITY_BIAS
        elif centre[axis] < shared_min + STABILITY_MARGIN:
            shift[axis] = STABILITY_BIAS

    return shift

def placed(env, item_pddl, name_mapping):
    """
    True if the brick rests near its benchmark target (it did not fall or tip).
    """
    target = benchmark_target(env, item_pddl)
    delta = env.data.body(name_mapping[item_pddl]).xpos - np.array(target["position"])

    return bool(np.hypot(delta[0], delta[1]) < PLACED_XY_TOL and abs(delta[2]) < PLACED_Z_TOL)


"""
Motion routines behind the three PDDL actions. Each returns True on success.
"""
def exec_pick(env, item_pddl, name_mapping):
    cand = feasible_config(env, item_pddl)

    if cand is None:
        print("PICK: no collision-free grasp for", item_pddl, flush=True)
        return False

    env_name = name_mapping[item_pddl]

    # Grasp pose of the hand body, derived from the brick's current pose.
    brick_pos = env.data.body(env_name).xpos.copy()
    grasp_pos = brick_pos + np.array([0.0, 0.0, HAND_ABOVE_BRICK])
    approach_pos = grasp_pos + APPROACH_OFFSET

    # Move to approach position. Of the two symmetric hand orientations the one
    # with the shorter joint-space path is used.
    q_current = env.data.qpos[:7].copy()
    grasp_quat, q_approach = None, None
    for quat in (cand.grasp_quat, flip_about_approach_axis(cand.grasp_quat)):
        q = tampanda.get_ik().solve(approach_pos, quat)
        if q is None:
            continue
        if q_approach is None or np.linalg.norm(q - q_current) < np.linalg.norm(q_approach - q_current):
            grasp_quat, q_approach = quat, q

    if q_approach is None: 
        print("PICK: approach IK failed for", item_pddl, flush=True)
        return False

    move_linear(env, q_approach)

    # Move to grasp position.
    move_vertical(env, approach_pos, grasp_pos, grasp_quat, tampanda)

    # Close the ee.
    close_gripper(env, grasp_width(item_pddl, cand))

    # Store transforms.
    brick_pos_held, brick_quat, hand_pos, hand_quat = get_brick_and_hand_pose(env, env_name)

    R_brick_actual = Rotation.from_quat(brick_quat[[1, 2, 3, 0]])
    R_hand_actual = Rotation.from_quat(hand_quat[[1, 2, 3 ,0]])

    held_relative_pos[item_pddl] = (R_brick_actual.inv().apply(hand_pos - brick_pos_held))
    held_relative_rot[item_pddl] = (R_brick_actual.inv() * R_hand_actual)

    # Lift the brick up.
    move_vertical(env, grasp_pos, grasp_pos + np.array([0.0, 0.0, LIFT_HEIGHT]), grasp_quat, tampanda)

    # The grasp only succeeded if the brick followed the hand.
    if env.data.body(env_name).xpos[2] - brick_pos[2] < 0.5 * LIFT_HEIGHT:
        print("PICK: brick was not lifted:", item_pddl, flush=True)
        return False

    return True

def exec_place(env, item_pddl, name_mapping):
    target_pos, target_rot = get_place_target(env, item_pddl)

    if target_pos is None:
        return False

    reached = place_at(env, item_pddl, target_pos, target_rot, name_mapping, tampanda, HOME_Q, APPROACH_OFFSET, open_gripper, held_relative_pos, held_relative_rot)

    return bool(reached) and placed(env, item_pddl, name_mapping)

def exec_stack(env, item_pddl, support, name_mapping):
    target_pos, target_rot = get_stack_target(env, item_pddl, support, name_mapping)

    if target_pos is None:
        return False

    target_pos += stability_bias(env, item_pddl, support)
    target_pos[2] -= PRESS_DEPTH
    reached = place_at(env, item_pddl, target_pos, target_rot, name_mapping, tampanda, HOME_Q, APPROACH_OFFSET, open_gripper, held_relative_pos, held_relative_rot, unload=PRESS_DEPTH)

    return bool(reached) and placed(env, item_pddl, name_mapping)


"""
DomainBridge: maps every PDDL action of domain.pddl to its motion routine and
tracks the symbolic state through the declared effects.
"""
def register_fluents(bridge, bricks):
    """
    All predicates are action-tracked. The initial truth values mirror the
    (:init ...) section written by parser.get_init().
    """
    bridge.fluent("hand_empty", initial=True)
    bridge.fluent("on_table", initial=[(b,) for b in bricks])
    bridge.fluent("clear", initial=[(b,) for b in bricks])
    bridge.fluent("pick_area", initial=[(b,) for b in bricks])
    bridge.fluent("holding")
    bridge.fluent("assembly_area")
    bridge.fluent("on_brick")

def register_actions(bridge, name_mapping):
    """
    One executor per PDDL action. Each returns (success, effects); DomainBridge
    applies the effects to its symbolic state only if the motion succeeded.
    The effect dictionaries are the :effect sections of domain.pddl.
    """
    @bridge.action("pick")
    def pick(env, fluents, b):
        return exec_pick(env, b, name_mapping), {
            ("pick_area", b): False,
            ("hand_empty",): False,
            ("holding", b): True,
            ("on_table", b): False,
        }

    @bridge.action("place")
    def place(env, fluents, b):
        return exec_place(env, b, name_mapping), {
            ("holding", b): False,
            ("assembly_area", b): True,
            ("hand_empty",): True,
            ("on_table", b): True,
        }

    @bridge.action("stack")
    def stack(env, fluents, b1, b2):
        return exec_stack(env, b1, b2, name_mapping), {
            ("holding", b1): False,
            ("assembly_area", b1): True,
            ("clear", b2): False,
            ("hand_empty",): True,
            ("on_brick", b1, b2): True,
        }


"""
Steps through the PDDL/UPF generated plan step by step.
Uses the registered DomainBridge actions.
""" 
def step_through_plan(bridge, plan, objects):
    """
    Executes the plan through DomainBridge. Before every action DomainBridge
    checks the PDDL preconditions against the tracked symbolic state
    (strict_preconditions). Execution stops at the first action that fails.
    """
    report = {"plan_completed": True, "steps_executed": 0, "failed_action": None, "failure_reason": None}

    for action in plan: 
        action_name = action.action.name 
        params = [str(param) for param in action.actual_parameters]

        try:
            success, _ = bridge.execute_action(action_name, *params, objects=objects)
            reason = "executor reported failure"
        except RuntimeError as error:
            success, reason = False, str(error)

        if not success:
            report.update(plan_completed=False, failed_action=f"{action_name}({', '.join(params)})", failure_reason=reason)
            break

        report["steps_executed"] += 1

    return report


"""
Function called in main.
Executes entire plan and returns result according to LegoSim benchmark.
"""
def execute_plan(plan, yaml_path, output_dir=None, seed=42):
    global env
    global name_mapping 
    global BLOCKS 
    global tampanda 
    global grasp_planner 
    global bridge 
    global held_relative_pos
    global held_relative_rot
    global HOME_Q

    GRASP_CONFIGS.clear()

    yaml_path = Path(yaml_path)
    if output_dir is None:
        output_dir = Path("runs") / f"{yaml_path.parent.name}_{yaml_path.stem}_seed{seed}"
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # The converted product and all episode artifacts of lego_sim go into the
    # run directory, so the task folder stays clean and runs do not overwrite
    # each other.
    new_yaml_path = output_dir / (yaml_path.stem + "_converted.yaml")
    convert_workbenchmark_yaml(yaml_path, new_yaml_path)
    env = LegoBenchEnv(new_yaml_path, seed=seed, output_dir=output_dir)

    HOME_Q = env.data.qpos[:7].copy()

    tampanda = LegoTampanda(env)

    name_mapping = generate_name_mapping(env, yaml_path)

    BLOCKS = list(name_mapping.keys())

    # GraspPlanner only contributes the two top-down orientation candidates.
    # table_z is the table body's centre, not its surface: GraspPlanner's
    # clearance check assumes tampanda's own gripper model and would reject
    # every top-down grasp of a 19 mm brick if given the real table top.
    TABLE_Z = env.data.body("table").xpos[2]
    grasp_planner = GraspPlanner(table_z=TABLE_Z)

    held_relative_pos = {}
    held_relative_rot = {} 

    bridge = DomainBridge(DOMAIN_PATH, env, strict_preconditions=True)
    register_fluents(bridge, BLOCKS)
    register_actions(bridge, name_mapping)

    generate_grasps(env, BLOCKS, grasp_planner, name_mapping)

    report = step_through_plan(bridge, plan, {"brick": BLOCKS})

    res = env.export_result()
    res.update(report)
    env.close()

    return res
