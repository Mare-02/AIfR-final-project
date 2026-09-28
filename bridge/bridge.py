import numpy as np
import yaml
from AI_module import LegoTampanda, convert_workbenchmark_yaml, move_linear, move_vertical, get_brick_and_hand_pose, place_at, get_place_target, get_stack_target
from tampanda.controllers.position_controller import PositionController
from tampanda.planners.grasp_planner import GraspType
from tampanda import DomainBridge, GraspPlanner
from parser import get_brick_names, block_rename
from mj_bridge.gym_env import LegoBenchEnv 
from scipy.spatial.transform import Rotation 

GRASP_CONFIGS = {}
GRASP_OFFSET = np.array([0, 0, 0.020])
APPROACH_OFFSET = np.array([0, 0, 0.15])
BLOCKS = []
HOME_Q = None 
HALF_SIZE = np.array([0.016, 0.008, 0.0096])

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
def corridor_blocked(env, other, block, gtype):
    """
    True if "other" sits in "block"'s gripper corridor for a "gtype" top-down grasp.
    TOP_DOWN_X closes along X (block if dx < CLEAR and dy < ALIGN);
    TOP_DOWN_Y closes along Y (block if dy < CLEAR and dx < ALIGN).
    """
    env_other = name_mapping[other]
    env_block = name_mapping[block]

    x_other = env.data.body(env_other).xpos.copy()[0]
    x_block = env.data.body(env_block).xpos.copy()[0]
    y_other = env.data.body(env_other).xpos.copy()[1]
    y_block = env.data.body(env_block).xpos.copy()[1]

    dx = abs(x_other - x_block)
    dy = abs(y_other - y_block)

    if gtype == GraspType.TOP_DOWN_X:
        if (dx < CLEAR) and (dy < ALIGN):
            return True 
    elif gtype == GraspType.TOP_DOWN_Y:
        if (dy < CLEAR) and (dx < ALIGN):
            return True

    return False 

def eval_ik_feasible(env, fluents, config: str) -> bool:
    """
    True iff no other block is in this config's grasp corridor.
    """
    block = GRASP_CONFIGS[config]["block"]
    grasp_type = GRASP_CONFIGS[config]["candidate"].grasp_type

    for entry in BLOCKS:
        if entry != block:
            if corridor_blocked(env, entry, block, grasp_type):
                return False 
    return True 

def feasible_config(env, item):
    """ 
    A grasp candidate for "item" that is collision-free in the current scene (or None).
    """
    for c in GRASP_CONFIGS:
        if GRASP_CONFIGS[c]["block"] == item and eval_ik_feasible(env, {}, c):
            return GRASP_CONFIGS[c]["candidate"]
    return None


def generate_grasps(env, blocks, grasp_planner, name_mapping):
    for block in blocks: 
        env_name = name_mapping[block]

        pos = env.data.body(env_name).xpos.copy()
        quat = env.data.body(env_name).xquat.copy()

        candidates = grasp_planner.generate_candidates(pos, HALF_SIZE, quat)
        i = 0
        for candidate in candidates:
            GRASP_CONFIGS[f"cfg_{block}_{i}"] = {"block": block, "candidate": candidate}
            i += 1 






"""
Functions used to open/close gripper.
"""
def close_gripper(env):
    action = env.data.ctrl.copy()
    action[7:9] = 0.014
    for _ in range(10):
        env.step(action)
def open_gripper(env):
    action = env.data.ctrl.copy()
    action[7:9] = 0.04 

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


def exec_pick(env, item_pddl, name_mapping):
    cand = feasible_config(env, item_pddl)

    if cand is None:
        return 

    env_name = name_mapping[item_pddl]

    # Move to approach position.
    approach_pos = cand.approach_pos + APPROACH_OFFSET 
    q_approach = tampanda.get_ik().solve(approach_pos, cand.grasp_quat)

    if q_approach is None: 
        return 

    move_linear(env, q_approach)

    # Move to grasp position.
    target_pos = cand.grasp_pos + GRASP_OFFSET 
    move_vertical(env, approach_pos, target_pos, cand.grasp_quat, tampanda)

    # Close the ee.
    close_gripper(env)

    # Store transforms.
    brick_pos, brick_quat, hand_pos, hand_quat = get_brick_and_hand_pose(env, env_name)

    R_brick_actual = Rotation.from_quat(brick_quat[[1, 2, 3, 0]])
    R_hand_actual = Rotation.from_quat(hand_quat[[1, 2, 3 ,0]])

    held_relative_pos[item_pddl] = (R_brick_actual.inv().apply(hand_pos - brick_pos))
    held_relative_rot[item_pddl] = (R_brick_actual.inv() * R_hand_actual)
    # Lift the brick up.
    start_pos = cand.grasp_pos + GRASP_OFFSET 
    target_pos = cand.lift_pos 
    move_vertical(env, start_pos, target_pos, cand.grasp_quat, tampanda)

def exec_place(env, item_pddl, name_mapping):
    target_pos, target_rot = get_place_target(env, item_pddl)
    place_at(env, item_pddl, target_pos, target_rot, name_mapping, tampanda, HOME_Q, APPROACH_OFFSET, open_gripper, held_relative_pos, held_relative_rot)

def exec_stack(env, item_pddl, support, name_mapping):
    target_pos, target_rot = get_stack_target(env, item_pddl, support, name_mapping)

    if target_pos is None:
        return 
    target_pos[2] -= 0.002
    place_at(env, item_pddl, target_pos, target_rot, name_mapping, tampanda, HOME_Q, APPROACH_OFFSET, open_gripper, held_relative_pos, held_relative_rot)



"""
Steps through the PDDL/UPF generated plan step by step.
Uses the registered DomainBridge actions.
""" 
def step_through_plan(env, plan, name_mapping):
    for action in plan: 
        action_name = action.action.name 

        params = [str(param) for param in action.actual_parameters]

        if action_name == "pick":
            exec_pick(env, params[0], name_mapping)

        if action_name == "place":
            exec_place(env, params[0], name_mapping)

        if action_name == "stack":
            exec_stack(env, params[0], params[1], name_mapping)



"""
Function called in main.
Executes entire plan and returns result according to LegoSim benchmark.
"""
def execute_plan(plan, yaml_path):
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

    new_yaml_path = yaml_path.with_name(yaml_path.stem + "_converted.yaml")
    convert_workbenchmark_yaml(yaml_path, new_yaml_path)
    env = LegoBenchEnv(new_yaml_path, seed = 42)

    HOME_Q = env.data.qpos[:7].copy()

    tampanda = LegoTampanda(env)

    name_mapping = generate_name_mapping(env, yaml_path)

    BLOCKS = list(name_mapping.keys())
    TABLE_Z = env.data.body("table").xpos[2]
    grasp_planner = GraspPlanner(table_z=TABLE_Z)

    held_relative_pos = {}
    held_relative_rot = {} 

    bridge = DomainBridge("./domain.pddl", env)
    bridge.action("pick")(exec_pick)
    bridge.action("place")(exec_place)
    bridge.action("stack")(exec_stack)

    generate_grasps(env, BLOCKS, grasp_planner, name_mapping)

    step_through_plan(env, plan, name_mapping)

    res = env.export_result()

    return res