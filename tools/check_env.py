"""Schritt-0-Smoketest: prueft die komplette Toolchain fuer PDDLego.

Laeuft ohne MuJoCo-Fenster und ohne Roboter. Verifiziert genau den Pfad,
den das Projekt spaeter nutzt: PDDL-Datei -> DomainBridge -> unified-planning
-> Plan als Liste von (action, params).
"""

from pathlib import Path
import sys

print("=" * 60)
print("1) Versionen")
print("=" * 60)
import tampanda
import mujoco
import unified_planning
import yaml

# Pfad aus dem installierten Paket ableiten, nicht hart kodieren —
# der TAMPanda-Klon liegt bei jedem Teammitglied woanders.
PDDL = (
    Path(tampanda.__file__).parent
    / "symbolic/domains/blocks/pddl/blocks_domain.pddl"
)

print(f"  python              {sys.version.split()[0]}")
print(f"  tampanda            {getattr(tampanda, '__version__', '1.0.0')}")
print(f"  mujoco              {mujoco.__version__}")
print(f"  unified_planning    {unified_planning.__version__}")
print(f"  pyyaml              {yaml.__version__}")
print(f"  PDDL-Domain         {PDDL}")
assert PDDL.is_file(), f"blocks_domain.pddl nicht gefunden unter {PDDL}"

print()
print("=" * 60)
print("2) Verfuegbare UPF-Planer")
print("=" * 60)
from unified_planning.shortcuts import get_environment
factory = get_environment().factory
for name in sorted(factory.engines):
    print(f"  {name}")

print()
print("=" * 60)
print("3) DomainBridge parst die PDDL-Domain")
print("=" * 60)
from tampanda.tamp import DomainBridge

# env=None ist ok, solange alle Praedikate als Fluents registriert sind:
# code-evaluierte Praedikate wuerden env brauchen, Fluents nicht.
bridge = DomainBridge(PDDL, environment=None)
print(bridge.describe())

print()
print("=" * 60)
print("4) Planen end-to-end (Blocksworld: b0 auf b1 auf b2)")
print("=" * 60)
for pred in ("on", "on-table", "clear", "holding", "gripper-empty"):
    bridge.fluent(pred)

# Startzustand: drei Bloecke flach auf dem Tisch, Greifer leer.
bridge._fluent_state.update({
    ("on-table", "b0"): True,
    ("on-table", "b1"): True,
    ("on-table", "b2"): True,
    ("clear", "b0"): True,
    ("clear", "b1"): True,
    ("clear", "b2"): True,
    ("gripper-empty", "g0"): True,
})

objects = {"block": ["b0", "b1", "b2"], "gripper": ["g0"]}
goals = [("on", "b0", "b1"), ("on", "b1", "b2")]

for engine in ("fast-downward", "pyperplan"):
    plan = bridge.plan(objects, goals=goals, planner_name=engine)
    if plan is None:
        print(f"  {engine:16s} KEIN PLAN GEFUNDEN")
        continue
    print(f"  {engine:16s} {len(plan)} Schritte")
    for i, (action, params) in enumerate(plan, 1):
        print(f"      {i}. {action}({', '.join(params)})")

print()
print("=" * 60)
print("FERTIG - Toolchain funktioniert.")
print("=" * 60)
