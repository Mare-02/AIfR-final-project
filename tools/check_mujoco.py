"""Schritt-0-Smoketest Teil 2: MuJoCo-Szene headless bauen und Zustand grounden.

Kein Viewer, kein Roboter-Motion. Verifiziert SceneBuilder -> FrankaEnvironment
-> make_blocks_bridge -> ground_state, also den Perzeptions-/Grounding-Pfad.
"""

print("Baue Blocks-Szene headless ...")
from tampanda.symbolic.domains.blocks.env_builder import make_blocks_builder
from tampanda.symbolic.domains.blocks.blocks_bridge import make_blocks_bridge

env = make_blocks_builder().build_env(rate=200.0)
print(f"  MuJoCo-Modell geladen: {env.model.nbody} bodies, {env.model.ngeom} geoms")

# executor=None und planner/grasp_planner=None -> keine Roboterbewegung,
# nur Praedikat-Grounding.
bridge, objects = make_blocks_bridge(env, block_indices=[0, 1, 2, 12])
print()
print(bridge.describe())

print()
print("Grounded state (nur True-Fakten):")
state = bridge.ground_state(objects)
true_facts = {k: v for k, v in state.items() if v}
if not true_facts:
    print("  (leer - Bloecke sind noch off-screen geparkt, das ist erwartbar)")
for key in sorted(true_facts):
    print(f"  {key}")

print()
print(f"Objekte: {objects}")
print()
print("OK - MuJoCo + Scene + Grounding funktionieren.")
