# SUCCESSOR_CONTEXT_OF_USE

MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`
LINEAR_ISSUE: `RES-82`
STATUS: **FROZEN BY THIS MISSION** (pending the explicit owner decisions listed in `OWNER_DECISIONS_REQUIRED.md`)
ENTRY_HEAD: `12ea42029489ddc2839389beafcf0811b7b45271`
ENTRY_TREE: `79f183724486ff95a1269a87cdb21d078b1893a5`

This document freezes the context of use (COU) for the successor nominal 20 kg loaded
countermovement-jump (CMJ) scientific task **before any successor Plant or controller is
tuned**. It is the upstream artifact for every gate in `SUCCESSOR_TASK_CONTRACT.md`.
The V&V principle applied is ASME V&V 40-2018 / FDA 2023 credibility logic: the
credibility requirements and the strength of any claim must be commensurate with the
decision the model is used to support. A deterministic simulation qualification under a
frozen model is **not** experimental validation.

---

## 1. TASK

**Nominal bilateral 20 kg externally loaded countermovement jump.**

- A single athlete performs one maximal-effort bilateral CMJ while carrying a 20 kg
  external load, starting from quiet bilateral standing on a level floor and ending in
  quiet bilateral standing.
- "Loaded" means one rigid 20 kg external body attached to the athlete model, unless a
  successor Plant (RES-83) changes the load embodiment; if the embodiment changes, the
  COU must be re-adjudicated by the owner before any qualification claim.
- The task is bilateral: both feet start planted and both feet must be re-established at
  landing. Slight left-right asynchrony at landing is a declared tolerance question
  (OD-10), not an assumed symmetry.
- The movement is sagittal-dominant: the model is a sagittal-reduced bilateral plant.
  Frontal-plane and transverse-plane behaviour is a declared model limitation, not a
  task requirement.

## 2. ATHLETE

- **Generic nominal adult athlete model** — 75 kg athlete + 20 kg load = 95 kg system.
- NOT subject-specific. NOT an anthropometric clone of any person. The inertias and
  segment properties are those of the frozen/nominal model and must be declared with
  provenance by RES-83.
- NOT population-representative. Mean values in the evidence table are used only to
  bound the *plausibility of the task*, never to predict an individual or a population.

## 3. LOAD

- 20 kg rigid external load on the nominal model (21.05 % of system mass,
  26.67 % of athlete mass), placed as declared by the Plant.
- The load's mass, geometry, attachment, and collision policy are Plant properties
  (RES-83). The scientific contract treats the load as a declared part of the system.

## 4. PLANE

- Sagittal-dominant reduced model (root translation x/z + pitch, plus 7 sagittal joints)
  unless RES-83 changes the representation.
- **Horizontal quantities that the contract checks are sagittal-plane quantities**
  (forward/backward `x` and vertical `z`), consistent with the model.
- Medio-lateral stability is NOT claimed. Any future medio-lateral claim requires a
  representation change and a new COU.

## 5. PURPOSE

Produce a **deterministic, physically credible, nominal 20 kg loaded-CMJ trajectory**
suitable for:

- research and software development on the loaded-CMJ task;
- verification / validation / uncertainty-quantification (VVUQ) development;
- event-contract, measurement-contract, scorer-contract, and negative-control
  development;
- control-qualification experiments under the frozen model.

## 6. NOT PURPOSE

This contract, and any PASS under it, does **not** support:

- clinical diagnosis or screening;
- injury-risk prediction (ACL, ankle, spine, or any other);
- individual athlete prescription, technique coaching, or load prescription;
- population normative prediction or benchmarking;
- neuromuscular or muscle-force prediction (there is no muscle model);
- neural control or motor-learning claims;
- optimal human technique claims;
- elite-performance benchmarking;
- experimental predictive validity of the computational model toward humans.

## 7. SUCCESS CLAIM MUST MEAN

> The simulated movement satisfies a **predeclared** physical, kinematic, performance,
> landing, balance, recovery, and numerical contract under the frozen nominal model and
> environment.

Success is a **qualified simulation result**, not a statement about a human.

## 8. WHAT A SUCCESSOR PASS MAY AND MAY NOT IMPLY

| Statement | Allowed by a successor PASS? |
|---|---|
| "The deterministic simulation executed and satisfied all declared gates." | YES |
| "The simulation produced a physically credible nominal loaded-CMJ trajectory under the declared model." | YES, if L1/L4/L5 gates pass |
| "The model reproduces measured human loaded-CMJ kinetics/kinematics within a tolerance." | NO — requires a validation referent |
| "The trajectory is a valid prediction of any real athlete." | NO |
| "The landing is safe / injury-free." | NO |
| "The technique is optimal." | NO |

## 9. V&V POSITION

- `verification` ≠ `validation`.
- `determinism` ≠ `biomechanical validity`.
- `event-chain completion` ≠ `task success` (this is the R001 defect class).
- `visual plausibility` ≠ `experimental validation`.

A future candidate may pass this simulation task **without** being experimentally
validated against humans. Every claim that requires experimental evidence is listed in
`SCIENTIFIC_CLAIM_CEILING.md`.

**Model-risk classification (ASME V&V 40 logic).** The decision supported by this
model is a research/software qualification decision with **low model risk**: no
clinical, regulatory, safety, or resource-allocation consequence follows from an
incorrect PASS, and the claim ceiling forbids human-predictive use. Credibility
requirements are therefore set per layer (L0/L1 code/solution verification; L2–L5
contract conformance; L6 provenance/determinism/review) and are deliberately not
raised to a medical-device evidence level. Raising the claim ceiling requires raising
the risk classification and the credibility evidence; medical-device risk thresholds
are not imported into this COU.

**Normative weight convention.** `BW` means the total system weight
(`m_system = 95.0 kg`, `g = 9.81 m/s²`, `BW = 931.95 N`). Athlete-only weight is never
used without an explicit qualifier.

## 10. COU CHANGE CONTROL

Any of the following invalidates the frozen COU and requires a new owner-approved COU
before qualification:

1. change of athlete mass model beyond declared tolerances;
2. change of external load mass or attachment beyond declared tolerances;
3. change of model representation (e.g., adding frontal-plane DoFs, MTP joints,
   muscle actuation, or a different root topology);
4. change of the declared task (e.g., loaded squat jump, drop jump, unilateral jump,
   non-maximal effort, arm swing);
5. expansion of the claim beyond the ceiling in `SCIENTIFIC_CLAIM_CEILING.md`.

## 11. COU PROVENANCE

| Item | Source class |
|---|---|
| Deterministic MuJoCo reduced sagittal nominal loaded-CMJ system | PROJECT HISTORICAL EVIDENCE (frozen V2/V2.1 model; RES-80 audit) |
| 75 kg athlete + 20 kg load, 95 kg system | PROJECT HISTORICAL EVIDENCE (V2 constants) |
| Verification/validation distinction and claim ceiling | PROJECT INTERNAL AUTHORITY (`PROJECT_SCIENTIFIC_CONSTITUTION.md`, `VVUQ_AND_QUALIFICATION_TAXONOMY.md`, `INTENDED_USE_AND_CLAIM_BOUNDARY.md`) |
| Credibility-commensurate-with-use principle | WEB/EXTERNAL LITERATURE (ASME V&V 40-2018; FDA 2023 CM&S credibility guidance) |
| Loaded-CMJ task identity and performance plausibility bands | WEB/EXTERNAL LITERATURE (see `SCIENTIFIC_CONTRACT_EVIDENCE_TABLE.md`) |

## 12. R001 RELATION

R001 remains a **canonical negative example**: it completed all twelve event labels
under the historical system while failing the owner-observed physical-visual
credibility review. Under the successor contract it is expected to FAIL
(`R001_SUCCESSOR_CONTRACT_RESULT=FAIL`), and the event-chain-completion-is-not-success
distinction is preserved as a required negative control (NC-10).
