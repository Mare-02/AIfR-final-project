import yaml
import os 
import sys 


flags = sys.argv 

input_path = flags[1]
output_path = flags[2]


with open(input_path, "r") as f:
            yaml_task = yaml.safe_load(f)

target = {b["name"]: b for b in yaml_task["blocks"]}

height_steps = 0.0191

def get_task_name(yaml_path: str):
    f = open(yaml_path)
    return os.path.basename(f.name)[:-5]


task_name = get_task_name(input_path)



def block_rename(block_name: str):
    return block_name[4:]+"_"+block_name[:3]

def get_brick_names(yaml_path: str):
    """
    Input:
    yaml_path: Path to the corresponding YAML file.

    Output: Single string object, formatted with the names of all bricks (letter first) and space inbetween names.
    """
    with open(yaml_path, "r") as f:
        task = yaml.safe_load(f)

    brick_set = [entry["name"] for entry in task["initial_blocks"]]

    brick_set = list(map(lambda x: block_rename(x), brick_set))
    
    return " ".join(brick_set)
    

brick_names = get_brick_names(input_path)

def get_init(brick_set: str):
    brick_list = brick_set.split()
    res = ""

    for entry in brick_list:
        res += "    (on_table " + entry + ")\n"
        res += "    (clear " + entry + ")\n"
        res += "    (pick_area " + entry + ")\n"

    res += "    (hand_empty)"

    return res


inits = get_init(brick_names)



def get_table(brick_list: list, target:dict):
    """
    Just checks for any bricks that have a height of 0.
    These bricks need to be on_table.
    """
    res = []

    for entry in target:
         if target[entry]["pos"][2] == 0:
              res.append("    (on_table " + block_rename(entry) + ")")
              
    return res

def get_stack(brick_list: list, target:dict):
    """
    This only works for Tier 1 and Tier 2.
    Here we can assume that a greater height means one block is stacked on a specific other block.
    Need further constraints for higher Tiers (e.g., x and y-coordinates).
    """
    res = [] 

    for entry1 in target:
         for entry2 in target:
                diff = (target[entry1]["pos"][2] - target[entry2]["pos"][2])
                if diff == height_steps:
                   res.append("    (on_brick " + block_rename(entry1) + " " + block_rename(entry2) + ")")

    return res

def get_goals(brick_set: str, target:dict):
    brick_list = brick_set.split()
    res = ""

    for entry in brick_list:
        res += "    (assembly_area " + entry + ")\n"

    table_list = get_table(brick_list, target)
    stack_list = get_stack(brick_list, target)

    table_list = "\n".join(table_list)
    stack_list = "\n".join(stack_list)

    res += table_list 
    res += "\n"
    res += stack_list

    return res


goals = get_goals(brick_names, target) 

template = [
   f"(define (problem {task_name})",
    "  (:domain lego)",
    "",
    "  (:objects", 
   f"    {brick_names} - brick",
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


result = "\n".join(template)

with open(output_path + "\\" + task_name + ".pddl", "w") as text_file:
    text_file.write(result)