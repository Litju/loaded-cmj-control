"""RES-86 audit-only reduced-foot model-form sensitivity tests.

MISSION: RES86_TOUCHDOWN_CONFIGURATION_AND_ABSORPTION_RESOLUTION_001
LINEAR ISSUE: RES-86

The audit realization releases the sealed locked midfoot into a bounded
midtarsal hinge *in memory only*.  These tests bind the audit guards: the
sealed XML is never mutated, the production Plant still rejects the variant,
the realization carries exactly the declared bounded freedom, and the
nominal-launch -> audit-Plant state transfer is exact.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
for _path in (str(TASK_ROOT), str(TASK_ROOT / "src")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from loaded_cmj.v3.plant import V3Plant, V3PlantError, model_xml  # noqa: E402
from tools.res86.reduced_foot_sensitivity import (  # noqa: E402
    LATTICE,
    MIDFOOT_JOINT_NAMES,
    NOMINAL,
    build_reduced_foot_plant,
    transfer_state,
)


def test_audit_realization_never_mutates_the_sealed_xml():
    xml_before = model_xml()
    build_reduced_foot_plant(LATTICE[1])
    build_reduced_foot_plant(LATTICE[3])
    assert model_xml() == xml_before


def test_audit_realization_adds_only_the_declared_bounded_midfoot_dof():
    plant = build_reduced_foot_plant(LATTICE[2])
    assert int(plant.model.njnt) == int(V3Plant().model.njnt) + 2
    import mujoco
    for name in MIDFOOT_JOINT_NAMES:
        jid = int(mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_JOINT, name))
        assert jid >= 0
        assert int(plant.model.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_HINGE)
        assert plant.model.jnt_limited[jid] == 1
        low, high = plant.model.jnt_range[jid]
        assert abs(low + LATTICE[2].midfoot_range_rad) < 1e-12
        assert abs(high - LATTICE[2].midfoot_range_rad) < 1e-12


def test_production_plant_still_rejects_the_audit_variant():
    plant = build_reduced_foot_plant(LATTICE[1])
    with pytest.raises(V3PlantError):
        V3Plant(model=plant.model)


def test_nominal_state_transfer_is_exact():
    nominal = V3Plant()
    data = nominal.make_data()
    data.qpos[nominal.idx.qadr["left_knee"]] = 0.7
    data.qpos[nominal.idx.qadr["left_ankle"]] = 0.2
    data.qvel[nominal.idx.vadr["left_knee"]] = 1.5
    data.ctrl[3] = -42.0
    data.time = 1.25
    import mujoco
    size = int(mujoco.mj_stateSize(nominal.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    vector = np.zeros(size, dtype=np.float64)
    mujoco.mj_getState(nominal.model, data, vector,
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    transferred = transfer_state(nominal, nominal, vector)
    assert np.array_equal(transferred, vector)


def test_nominal_realization_is_the_locked_midfoot_control():
    plant = build_reduced_foot_plant(NOMINAL)
    assert int(plant.model.njnt) == int(V3Plant().model.njnt)
    assert NOMINAL.midfoot_range_rad == 0.0
