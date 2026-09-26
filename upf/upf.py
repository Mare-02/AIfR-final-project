import itertools
from unified_planning.io import PDDLReader 
from typing import List, Optional, Set
from collections import deque
from pathlib import Path



"""
Helper functions from the exercise sheets.
"""
def evaluate_node(node, state) -> bool:
    if node.is_true(): return True 
    if node.is_false(): return False 
    if node.is_and(): return all(evaluate_node(a, state) for a in node.args)
    if node.is_or(): return any(evaluate_node(a, state) for a in node.args)
    if node.is_fluent_exp():
        return str(node) in state
    return True 

def get_add_effects(action_instance) -> Set[str]:
    """Return ground atom strings that this grounded action adds."""
    subst = dict(zip(action_instance.action.parameters, action_instance.actual_parameters))
    return {str(eff.fluent.substitute(subst))
            for eff in action_instance.action.effects if eff.value.is_true()}

def get_del_effects(action_instance) -> Set[str]:
    """Return ground atom strings that this grounded action deletes."""
    subst = dict(zip(action_instance.action.parameters, action_instance.actual_parameters))
    return {str(eff.fluent.substitute(subst))
            for eff in action_instance.action.effects if eff.value.is_false()}

def state_from_problem(problem) -> Set[str]:
    """Extract the initial state as a set of ground-atom strings from a UPF problem."""
    return {str(f) for f, val in problem.explicit_initial_values.items()
            if str(val) == "true"}

def _collect_positive(node, out: Set) -> None:
    """Collect all positive fluent-expression strings from an FNode tree."""
    if node.is_fluent_exp():
        out.add(str(node))
    elif node.is_and():
        for a in node.args:
            _collect_positive(a, out)

def goal_atoms(problem) -> Set[str]:
    """Return the goal as a set of ground-atom strings (positive literals only)."""
    atoms: set[str] = set()
    for goal in problem.goals:
        _collect_positive(goal, atoms)
    return atoms

def action_name(action_instance) -> str:
    """Return a human-readable name for a grounded action instance."""
    args = ", ".join(str(a) for a in action_instance.actual_parameters)
    return f"{action_instance.action.name}({args})"


"""
Functions defined as part of the exercise sheets.
"""
def ground_actions(problem) -> list:
    grounded_instances = []

    for a in problem.actions:
        parameter_objects = []

        for p in a.parameters:
            objects = [obj for obj in problem.all_objects if obj.type == p.type]
            parameter_objects.append(objects)

        for combination in itertools.product(*parameter_objects):
            grounded_instances.append(a(*combination))

    return grounded_instances



def action_applicable(action, state) -> bool:
    subst = dict(zip(action.action.parameters, action.actual_parameters))
    
    for fnode in action.action.preconditions:
        if not evaluate_node(fnode.substitute(subst), state):
            return False

    return True 


def apply_action(action, state) -> Set[str]:
    return (state - get_del_effects(action)) | get_add_effects(action)    


"""
BFS planner defined during the exercise sheets.
"""
def strips_planner(problem) -> Optional[List]:
    """
    BFS forward planner for a UPF STRIPS problem.

    Parameters
    ----------
    problem : unified_planning Problem

    Returns
    -------
    List of ActionInstance (plan), or None if unsolvable.
    """
    initial = state_from_problem(problem)
    goals   = goal_atoms(problem)
    actions = ground_actions(problem)

    BFS_queue = deque([(initial, [])])
    visited_states = set()
    
    while BFS_queue:
        # Look at the right state of the queue.
        # Remember it to not visit it again later.
        curr_entry = BFS_queue.pop()
        visited_states.add(frozenset(curr_entry[0]))

        # Test if goal state was reached via subset relation.
        if goals <= curr_entry[0]:
            return curr_entry[1]
        
        # Collect all actions that are applicable.
        # If state has not been visited before, append left.
        for a in actions:
            if action_applicable(a, curr_entry[0]):
                new_state = apply_action(a, curr_entry[0])

                if frozenset(new_state) not in visited_states:
                    visited_states.add(frozenset(new_state))

                    new_path = curr_entry[1].copy()
                    new_path.append(a)

                    new_entry = (new_state, new_path)
                    BFS_queue.appendleft(new_entry)

    # If no path was found, return None.
    return None


"""
Generate the ordered plan.
"""
def generate_plan(domain: Path, problem: Path):
    reader = PDDLReader()
    problem = reader.parse_problem(domain, problem)
    return strips_planner(problem)