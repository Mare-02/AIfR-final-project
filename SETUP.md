# PDDLego — Entwicklungsumgebung

Reproduzierbare Umgebung für die Pipeline YAML → PDDL → Plan → Ausführung in
`lego_sim`. Verifiziert am 2026-09-30 unter macOS (arm64) mit Python 3.12.12.

## Voraussetzungen

- **Python ≥ 3.10** — TAMPanda verlangt das in `pyproject.toml`. Das System-Python
  von macOS (3.9.6) reicht **nicht**. Unter macOS: `brew install python@3.12`.
- Ein lokaler Klon von **TAMPanda**.
- Ein lokaler Klon von **lego_sim** (die Simulationsumgebung, die wir anstelle
  der noch nicht veröffentlichten WorkBenchMark-Umgebung verwenden):

```bash
git clone https://github.com/ma-haha-hehe/lego_sim.git ../lego_sim
```

Die Auswertung in `evaluation/` wurde mit lego_sim-Commit `326d572` erzeugt.

Die Anleitung geht davon aus, dass `tampanda/` und `lego_sim/` **neben** diesem
Repo liegen. Andernfalls die Pfade unten anpassen.

## Aufgaben

Die 200 Aufgaben aus Tier 1 und Tier 2 liegen unter `Project/tasks/`. Es sind
unveränderte Kopien von `ground_truth/tier1` und `ground_truth/tier2` aus
https://github.com/WorkBenchMark/dataset (Commit `589b3b9`). Ein Submodul gibt
es nicht mehr.

## Einrichtung

```bash
python3.12 -m venv .venv
```

```bash
./.venv/bin/python -m pip install --upgrade pip setuptools wheel
```

TAMPanda editierbar installieren:

```bash
./.venv/bin/python -m pip install -e "../tampanda"
```

Restliche Abhängigkeiten:

```bash
./.venv/bin/python -m pip install -r requirements.txt
```

`lego_sim` ist ein ROS-2-Paket und lässt sich nicht per `pip` installieren. Die
direkte Python-API (`mj_bridge.gym_env.LegoBenchEnv`) braucht aber kein ROS; es
genügt, das Paketverzeichnis in den Suchpfad der Umgebung einzutragen:

```bash
echo "$(cd ../lego_sim/src/mj_bridge && pwd)" > "$(./.venv/bin/python -c 'import site; print(site.getsitepackages()[0])')/lego_sim_mj_bridge.pth"
```

## Verifikation

```bash
./.venv/bin/python tools/check_env.py
```

Prüft Versionen, listet die verfügbaren UPF-Engines und löst ein
Blocksworld-Problem mit Fast Downward und pyperplan.

```bash
./.venv/bin/python Project/main.py --tier 1 --task 001
```

Plant und führt eine Aufgabe vollständig aus (ca. 35 s). Erwartete Ausgabe:
`Success: True (2/2 bricks within tolerance)`.

## Nicht im Repo enthalten

Zwei PDFs sind bewusst per `.gitignore` ausgeschlossen, weil sie nicht unser
Urheberrecht sind. Bei Bedarf lokal ablegen:

| Datei | Quelle |
|---|---|
| `2606.19358v2.pdf` | WorkBenchMark-Paper, https://arxiv.org/abs/2606.19358 |
| `final_projects.pdf` | Kursunterlage RWTH AI for Robotics SS 2026 (Moodle) |

`AI_Robotics_Proposal.pdf` ist unser eigenes Dokument und liegt im Repo.

## Fallstricke

- **`MUJOCO_GL=egl` schlägt unter macOS fehl.** Die Variable einfach nicht
  setzen — der Standard funktioniert.
- **lego_sim kopiert pro Lauf 33 MB Roboter-Meshes** in das Ausgabeverzeichnis.
  `evaluate.py` löscht sie nach jedem Lauf wieder; wer `main.py` oft von Hand
  startet, sollte `tmp/` gelegentlich leeren.
- **`pip install -e` mit lokalem Pfad ist nicht portabel.** Deshalb steht
  TAMPanda bewusst nicht in `requirements.txt` — jede:r installiert den eigenen
  Klon von Hand.
- **Die direkte Python-API von lego_sim hat keine Noppenklemmung.** Die steckt
  nur in der ROS-2-Bridge (`mj_bridge3.py`). Ein losgelassener Stein hält
  allein durch Schwerkraft und Kontakt. Was das für Tier 2 bedeutet, steht im
  Bericht und lässt sich mit `Project/static_stability.py` nachmessen.

## Versionen

| Komponente | Version |
|---|---|
| TAMPanda | 1.0.0 (editierbar) |
| lego_sim | Commit `326d572` |
| MuJoCo | 3.11.0 |
| unified-planning | 1.3.0 |
| Fast Downward (`up-fast-downward`) | 0.5.2 |
| pyperplan | 2.1 |
