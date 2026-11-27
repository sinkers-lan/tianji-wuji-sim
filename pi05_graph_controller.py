"""Source embedded as text in the USD Action Graph."""
import numpy as np
import omni.physics.tensors as tensors
import omni.timeline
import omni.usd


def initialize(state, context, ratios):
    sim = tensors.create_simulation_view("numpy", stage_id=context.get_stage_id())
    art = sim.create_articulation_view("/Workspace/Robot/Base")
    if art.count != 1 or art.max_dofs != 54:
        return False
    joint_names = [f"Joint{i}_{side}" for side in "LR" for i in range(1, 8)]
    indices = [art.shared_metatype.dof_names.index(name) for name in joint_names]
    stiffness = art.get_dof_stiffnesses()[0, indices]
    mass_matrix = art.get_generalized_mass_matrices()[0]
    inertia = 1 / np.diag(np.linalg.inv(mass_matrix))[indices]
    damping = art.get_dof_dampings().copy()
    damping[0, indices] = 2 * np.tile(ratios, 2) * np.sqrt(stiffness * inertia)
    state.articulation_indices = np.array([0], dtype=np.int32)
    art.set_dof_dampings(damping, state.articulation_indices)
    state.sim = sim
    state.art = art
    state.arm_indices = indices
    state.efforts = np.zeros_like(art.get_gravity_compensation_forces())
    print("PI05_USD_CONTROL_ACTIVE", flush=True)
    return True


def compute(db):
    if not omni.timeline.get_timeline_interface().is_playing():
        return True
    context = omni.usd.get_context()
    state = db.per_instance_state
    art = getattr(state, "art", None)
    if art is None or not art.check():
        if not initialize(state, context, db.inputs.dampingRatios):
            return True
        art = state.art
    inds = state.arm_indices
    state.efforts[0, inds] = art.get_gravity_compensation_forces()[0, inds]
    art.set_dof_actuation_forces(state.efforts, state.articulation_indices)
    return True
