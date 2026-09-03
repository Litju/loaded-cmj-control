# Intended Use and Claim Boundary

**Version:** 1.0.0  
**Date:** 2026-09-03  
**Constitution:** `PROJECT_SCIENTIFIC_CONSTITUTION.md`  
**Entry Head:** `2a5967d359f34562a9e356c3b138062cffdf4d51`  
**Status:** CANONICAL

---

## 1. Real-World System (RWS) — Human Loaded CMJ

**System:** A live human (~75 kg athlete load model, ~20 kg barbell/high-bar) performing a countermovement jump with unconstrained full-body neuromuscular control, soft-tissue compliance, variable anthropometry, fatigue, load placement, footwear/floor interaction, and coaching context.

**Real-world observables (not computed here):** 3-D kinematics, ground-reaction force via physical force plates, EMG, bar path, injury risk, performance outcome — requiring calibrated sensors, population sampling, and ethics-approved human-subjects protocols.

**Project stance:** The current computational system is *inspired by* the RWS but does not, by itself, predict individual or population human behavior. Any human-predictive claim requires an explicit, independent experimental referent and a formal `MODEL_VALIDATION` activity (see VVUQ taxonomy).

---

## 2. Computational System (CS) — Reduced MuJoCo Simulation

**System:** Deterministic sagittal-dominant rigid-body MuJoCo simulation:

- **Bodies:** pelvis, torso+arms, 20 kg cylindrical load, L/R thigh, shank, foot (10 total)
- **Joints:** `root_tx` (x-slide), `root_tz` (z-slide), `root_ry` (y-hinge) — all honestly floating (`limited=false`, zero stiffness/damping/armature) — plus 7 sagittal hinges (lumbar, 2× hip, 2× knee, 2× ankle), `nq=nv=10, nu=7`
- **Contacts:** bilateral `foot_box` (0.30×0.12×0.02 m, `solref 0.016 1.0` calibrated) ↔ floor (`condim 3`, `friction 0.9 ...`), 6 fall-shell capsules (contype 4, not plantar), floor `conaffinity 6`, `solimp 0.99 ...`
- **Actuation:** `tau = limit * u`, limits `[250,250,250,300,300,200,200]` Nm, fail-closed on nonfinite/OOB, no `qfrc_applied`, no hidden drive state
- **Numerics:** `implicitfast` + `Newton`, `dt=0.000125 s`, 40 substeps/control (`0.005 s`, 200 Hz), `iterations 100`, `ls_iterations 50`, `tolerance 1e-10`, `cone elliptic`, `impratio 10`, `gravity -9.81`, horizon `8.0 s` (1600 controls)
- **Measurement:** Virtual bilateral 6-axis plates, whole-wrench = sum(left+right+any other), COP valid only if `Fz>20 N` per foot, support margin from convex hull, prohibited contacts excluded from plantar counts
- **Events:** Monotone 12-gate DAG (`supported_start` → `stable_recovery`), physics-substep dwell, hysteresis, fall as terminal `PHYSICAL_FALL`, `INCOMPLETE_HORIZON` if gates not closed

**Identity pins (current authority):**

| Artifact | Authority | SHA256 |
|---|---|---|
| MJCF | `v2_plant.xml` | `5f2244149c6b8831a1bfe6fa29a8ce33be0e41100fbdd61bfd8d9555222be191` |
| Constants | `v2/constants.py` | `e6608f8eb8118befa27c603b5bb73ff1d263f4e4852b05cbdca9fd4336805756` |
| Plant code | `v2/plant.py` | `86b22181dd2fbddb864f483ce44f69036e7a28453660dd7f12bfb3dc05d9b52c` |
| Events | `v2/events.py` | `286ef328e4b334a15775df01ba6aad971cf8f808ddbcb028fcda4032164f2deb` |
| Controller | `v2/controller.py` (HEAD) | `870bab2676069a8fee927c637f3988a0ebbb535b42a1743fd51f2869adf436ed` |
| Drive | `v2/drive.py` | `02f30f1ccace3f2de109ca0fbe3a7da271bd437ae6b6b6a4757ce02dbaf6df21` |
| MuJoCo | 3.8.0 | `uv.lock` `a80b951e...` |
| Commit | `2a5967d` | tree `b7bec500...` |

Full ledger with dependencies/caveats: `AUTHORITY_LEDGER.json`.

---

## 3. Current Acceptable Claim (CAC)

> **Deterministic mechanically specified MuJoCo loaded-CMJ simulation and controller qualification under the frozen computational model.**

Specifically MAY claim, *when sealed with delivered evidence*:

- The CS Plant with the frozen MJCF/constants/solver is mass/inertia/topology-faithful (95.0 kg total, per-body closure) and contact/sensor/mechanics-closed.
- A named controller commit, from the frozen `qpos=[0,0.90,0,...]` standing reset, deterministically produces a specific 8.0 s trace, event-record sequence, and `RolloutResult` digest under the frozen `V2EventDetector` and `V2Plant` authorities, reproducible from the recorded initial `MjData` state in a fresh process.
- Landing, capture, or recovery gates are satisfied only if the 12-event DAG, dwell, bilateral, fall, penetration, chatter, and margin metrics meet their frozen thresholds — as independently recomputed from the raw trace.

All CAC claims are **scoped to the CS**. They state what the simulation does, not what humans do.

---

## 4. Currently NOT Justified (Claim Ceiling — Hard Boundary)

The following require independent validation referents that **do not currently exist** in this program, and must NOT be asserted as established facts:

1. **Human predictive validity** — no claim that the CS predicts CO M, GRF, kinematics, or outcomes of any real athlete or population.
2. **Subject-specific biomechanics** — no claim of anthropometric specificity beyond the nominal 75+20 kg with the documented segment inertias.
3. **Muscle / neural physiology** — no muscle model, activation dynamics, reflex, or neural control is present; torque is direct bounded `tau = limit * u`.
4. **Optimal human technique** — no optimality with respect to human performance, injury, or energetics is established; controller optima are CS optima only.
5. **Injury / safety prediction** — no claim that peak Fz, penetration, or joint loads predict injury risk.
6. **Empirical model validation** — no `MODEL_VALIDATION` has been performed; the CS has not been compared to independent force-plate / motion-capture referent data under a predeclared tolerance.

Any future claim in this list requires: (a) a named referent dataset with provenance, (b) a predeclared validation metric and tolerance, (c) an independent reconciliation, and (d) a sealed validation evidence bundle.

---

## 5. Allowed Adjacent Claims (with evidence)

- **Code verification:** Plant/solver/actuator/measurement/event code correctly implements the documented math (tests, closures, cross-checks).
- **Solution verification:** Numerical error controlled (`dt`, `tol`, determinism, residual bounds) for the reported trace.
- **Control qualification:** A controller meets the predeclared event/metric gates under the frozen CS (12/12 `OBJECTIVE_COMPLETE`, no fall, no artificial support).
- **Reproducibility:** Bit-identical replay from saved integration state in a fresh process.
- **Uncertainty characterization:** Stated sensitivity to declared parameter variations (not a full UQ study).
- **Scientific review:** Independent review of methods and evidence completeness.

These are distinct from `MODEL_VALIDATION` and must be labeled as such.

---

## 6. Use Constraints

- The software is **scientific simulation and control research**, not medical, clinical, coaching, or safety-critical software.
- Controller results must not be used to advise real human loading, technique, or progression without human-subjects review and independent validation.
- Evidence bundles are the authoritative record; receipts or Linear Done labels alone are not qualification.

---

## 7. Amendment

Expanding the claim ceiling requires a versioned amendment, a new validation referent, and a sealed validation bundle — not a prose assertion. The ceiling is intentionally conservative; relaxing it is a research program, not an edit.
