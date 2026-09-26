import yaml
import os 


HEIGHT_STEP = 0.0191


def get_task_name(input_path: str):
    """
    Input: 
    YAML file.

    Output: 
    Name of the YAML task, e.g. "task_001".
    """
    return os.path.splitext(os.path.basename(input_path))[0]


def block_rename(block_name: str):
    """
    Input:
    Block name in YAML format.

    Output:
    Block name in PDDL format.
    """
    return block_name[4:] + "_" + block_name[:3]


def get_brick_names(task: dict):
    """
    Input:
    YAML task.

    Output: 
    String object.
    Format: (PDDL) Names of all bricks (letter first) and space inbetween names.
    """
    brick_set = [entry["name"] for entry in task["initial_blocks"]]
    brick_set = list(map(block_rename, brick_set))
    
    return " ".join(brick_set)


def get_init(brick_set: str):
    """
    Input:
    String of all bricks.

    Output:
    Initializes String that expresses the following:
    All bricks are on the table at the start.
    All bricks are clear.
    All bricks are in the pick_area.
    Hand is empty.
    """
    brick_list = brick_set.split()

    res = []
    for entry in brick_list:
        res.append(f"    (on_table {entry})")
        res.append(f"    (clear {entry})")
        res.append(f"    (pick_area {entry})")
    res.append("    (hand_empty)")

    return "\n".join(res)


def get_table(target: dict):
    """
    Input:
    Dictionary of target positions for all bricks from the YAML file.

    Output:
    Initializes list that expresses the following:
    This brick has to be on the table at the goal state.
    """
    res = []
    for entry in target:
         if target[entry]["pos"][2] == 0:
              res.append("    (on_table " + block_rename(entry) + ")")
              
    return res


def get_stack(target:dict):
    """
    Input:
    Dictionary of target positions for all bricks from the YAML file.

    Output:
    Initializes list that expresses the following:
    Brick with greater height has to be stacked on brick with lower height.
    Exactly if the height discrepancy is equal to HEIGHT_STEP.
    
    But (!): this only works for Tier 1 and Tier 2.
    """
    res = [] 
    for entry1 in target:
         for entry2 in target:
                diff = (target[entry1]["pos"][2] - target[entry2]["pos"][2])
                if abs(diff - HEIGHT_STEP) < 1e-6:
                   res.append("    (on_brick " + block_rename(entry1) + " " + block_rename(entry2) + ")")

    return res


def get_goals(brick_set: str, target:dict):
    """
    Input:
    Set of all bricks and dictionary with all target positions.

    Output:
    Return String object that expresses the following in the PDDL context:
    Which brick has to be stacked on which brick at the goal state.
    Which brick has to be positioned in the assembly area at the goal state.
    Which brick has to be on the table at the goal state.
    """
    brick_list = brick_set.split()

    res = []

    for entry in brick_list:
        res.append(f"    (assembly_area {entry})")

    res.extend(get_table(target))
    res.extend(get_stack(target))

    return "\n".join(res)


def generate_template(
        name: str, 
        bricks: str, 
        inits : str, 
        goals: str):
    return [
        f"(define (problem {name})",
        "  (:domain lego)",
        "",
        "  (:objects", 
        f"    {bricks} - brick",
        "  )",
        "",
        "  (:init",
        f"{inits}",
        "  )",
        "",
        "  (:goal (and",
        f"{goals}",
        "  ))",
        ")"
    ]


def generate_pddl(input_path, output_path):
    # Read in YAML file.
    with open(input_path, "r") as f:
        yaml_task = yaml.safe_load(f)

    # Get all values needed for the PDDL problem file.
    task_name = get_task_name(input_path)
    target = {b["name"]: b for b in yaml_task["blocks"]}
    brick_names = get_brick_names(yaml_task)
    inits = get_init(brick_names)
    goals = get_goals(brick_names, target) 

    # Generate PDDL problem file content from template.
    template = generate_template(task_name, brick_names, inits, goals)
    res = "\n".join(template)

    # Save PDDL file at output path.
    with open(output_path, "w") as f:
        f.write(res)


