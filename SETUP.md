# PDDLego — Entwicklungsumgebung

Schritt 0 des Projekts: reproduzierbare Umgebung für TAMPanda + PDDL-Planung.
Verifiziert am 2026-07-30 unter macOS (arm64) mit Python 3.12.12.

## Voraussetzungen

- **Python ≥ 3.10** — TAMPanda verlangt das in `pyproject.toml`. Das System-Python
  von macOS (3.9.6) reicht **nicht**. Unter macOS: `brew install python@3.12`.
- Ein lokaler Klon von **TAMPanda**. Bei Marian liegt er unter
  `~/uni/Artificial Intelligence for Robotics/tampanda`.

## Repo klonen

Der Benchmark-Datensatz ist als **Submodul** eingebunden (gepinnt auf Commit
`589b3b9`). Ohne `--recursive` bleibt `dataset/` leer:

```bash
git clone --recursive <repo-url>
```

Schon geklont und `dataset/` ist leer? Dann nachziehen:

```bash
git submodule update --init --recursive
```

## Einrichtung

```bash
python3.12 -m venv .venv
```

```bash
./.venv/bin/python -m pip install --upgrade pip setuptools wheel
```

TAMPanda editierbar installieren — **Pfad an den eigenen Klon anpassen**:

```bash
./.venv/bin/python -m pip install -e "../tampanda"
```

Restliche Abhängigkeiten:

```bash
./.venv/bin/python -m pip install -r requirements.txt
```

## Verifikation

Beide Skripte müssen fehlerfrei durchlaufen.

```bash
./.venv/bin/python tools/check_env.py
```

Prüft Versionen, listet die verfügbaren UPF-Engines, lässt `DomainBridge` die
mitgelieferte Blocksworld-Domain parsen und löst ein Dreier-Stapel-Problem mit
**beiden** Planern. Erwartete Ausgabe: je ein 4-Schritte-Plan für
`fast-downward` und `pyperplan`.

```bash
./.venv/bin/python tools/check_mujoco.py
```

Baut die Blocks-Szene headless (32 bodies, 110 geoms), verdrahtet
`make_blocks_bridge` ohne Executor und groundet den Zustand. Kein Fenster, keine
Roboterbewegung.

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
  setzen — der Standard funktioniert. `build_env()` öffnet von sich aus kein
  Fenster; der Viewer ist ein separater `launch_viewer()`-Aufruf.
- **`.venv/` gehört nicht ins Git-Repo.** Sobald das Repo steht, in
  `.gitignore` aufnehmen.
- **`pip install -e` mit lokalem Pfad ist nicht portabel.** Deshalb steht
  TAMPanda bewusst nicht in `requirements.txt` — jede:r installiert den eigenen
  Klon von Hand.

## Was damit steht

| Komponente | Status |
|---|---|
| TAMPanda 1.0.0 (editierbar) | ✅ |
| MuJoCo 3.11.0, headless | ✅ |
| unified-planning 1.3.0 | ✅ |
| Fast Downward 0.5.2 | ✅ Plan verifiziert |
| pyperplan 2.1 | ✅ Plan verifiziert |
| `DomainBridge` PDDL → Plan | ✅ end-to-end |

Die UPF-Anbindung aus dem Proposal ist damit **nicht mehr zu bauen** —
`DomainBridge.plan()` erledigt sie bereits. Was bleibt: eigene PDDL-Domain,
YAML→Problem-Parser, eigene Prädikat-Evaluatoren und Action-Executors.
