from pathlib import Path 
from parser import generate_pddl
from bridge import execute_plan
from upf import generate_plan


def main():
    """
    Step 0.
    Create the PDDL Domain.
    I did that manually, can be found in "domain.pddl".
    """
    
    # :)
    
    """
    Step 1.
    Generate the PDDL file from the corresponding YAML file.
    For this I created the parser.py module with all necessary functions.
    """
    
    task_id = input("Please enter the Task-ID (e.g., 001 for task_001): ")
    tier_id = input("Please enter the Tier-ID: (e.g., 2 for Tier 2 tasks): ")

    yaml_path = Path("./tasks") / f"Tier_{tier_id}" / f"task_{task_id}.yaml"

    work_dir = Path("tmp")
    work_dir.mkdir(exist_ok=True)

    pddl_path = work_dir / f"task_{task_id}.pddl"

    generate_pddl(yaml_path, pddl_path)

    """    
    Step 2.
    Use the Universal Planning Framework to create an ordered plan.
    For this I created the upf.py module with all necessary functions.
    I have recycled some of the helper functions from the exercises we solved during the semester.
    """
    
    domain = Path("./domain.pddl")
    plan = generate_plan(domain, pddl_path)

    """
    Step 3. 
    Given: The PDDL domain, YAML (ground truth, for simplification), generated PDDL file and UPF plan.
    Use TamPanda/DomainBridge, the Lego_Sim Environment and WorkBenchMark to execute symbolic plan.
    """
    res = execute_plan(plan, yaml_path)

    """
    Step 4.
    Evaluate the executed plan.
    We of course do that here for the actual evaluation of our approach.
    """
    print(res)

if __name__ == "__main__":
    main()