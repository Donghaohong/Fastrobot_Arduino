import numpy as np


_BOUND_LOC = None
_ACTIVE_BELIEF_THRESHOLD = 1e-4
_EPS = 1e-9


def bind_context(loc_obj):
    """Bind a BaseLocalization instance so notebook wrappers can stay simple."""
    global _BOUND_LOC
    _BOUND_LOC = loc_obj


def _require_loc(loc_obj=None):
    if loc_obj is not None:
        return loc_obj
    if _BOUND_LOC is None:
        raise RuntimeError("Lab10 localization context is not bound. Call bind_context(loc).")
    return _BOUND_LOC


def _normalize_angle(angle_deg):
    return (np.asarray(angle_deg, dtype=float) + 180.0) % 360.0 - 180.0


def _as_pose_array(pose):
    pose = np.asarray(pose, dtype=float)
    if pose.shape[-1] != 3:
        raise ValueError("Pose must have three elements: (x, y, yaw_deg).")
    return pose


def _compute_control_vectorized(cur_poses, prev_pose):
    cur_poses = np.asarray(cur_poses, dtype=float)
    prev_pose = _as_pose_array(prev_pose)

    dx = cur_poses[..., 0] - prev_pose[0]
    dy = cur_poses[..., 1] - prev_pose[1]

    delta_trans = np.hypot(dx, dy)
    heading = np.degrees(np.arctan2(dy, dx))

    delta_rot_1 = _normalize_angle(heading - prev_pose[2])
    stationary = delta_trans < _EPS
    delta_rot_1 = np.where(stationary, 0.0, delta_rot_1)

    delta_rot_2 = _normalize_angle(cur_poses[..., 2] - prev_pose[2] - delta_rot_1)
    return delta_rot_1, delta_trans, delta_rot_2


def _state_pose_table(loc_obj):
    if not hasattr(loc_obj, "_lab10_state_pose_table"):
        mapper = loc_obj.mapper
        loc_obj._lab10_state_pose_table = np.column_stack(
            (
                mapper.x_values.reshape(-1),
                mapper.y_values.reshape(-1),
                mapper.a_values.reshape(-1),
            )
        )
    return loc_obj._lab10_state_pose_table


def compute_control(cur_pose, prev_pose):
    """Extract (rot1, trans, rot2) from two poses in (m, m, deg)."""
    cur_pose = _as_pose_array(cur_pose)
    prev_pose = _as_pose_array(prev_pose)

    delta_rot_1, delta_trans, delta_rot_2 = _compute_control_vectorized(
        cur_pose.reshape(1, 3), prev_pose
    )

    return float(delta_rot_1[0]), float(delta_trans[0]), float(delta_rot_2[0])


def odom_motion_model(cur_pose, prev_pose, u, loc_obj=None):
    """Return p(x_t | u_t, x_{t-1}) under the odometry motion model."""
    loc_obj = _require_loc(loc_obj)

    u = np.asarray(u, dtype=float)
    if u.shape[-1] != 3:
        raise ValueError("Control input u must have three elements: (rot1, trans, rot2).")

    rot1_hat, trans_hat, rot2_hat = compute_control(cur_pose, prev_pose)

    rot1_err = _normalize_angle(rot1_hat - u[0])
    trans_err = trans_hat - u[1]
    rot2_err = _normalize_angle(rot2_hat - u[2])

    prob = (
        loc_obj.gaussian(rot1_err, 0.0, loc_obj.odom_rot_sigma)
        * loc_obj.gaussian(trans_err, 0.0, loc_obj.odom_trans_sigma)
        * loc_obj.gaussian(rot2_err, 0.0, loc_obj.odom_rot_sigma)
    )
    return float(prob)


def prediction_step(cur_odom, prev_odom, loc_obj=None):
    """Update loc.bel_bar from loc.bel using the odometry motion model."""
    loc_obj = _require_loc(loc_obj)
    actual_u = np.asarray(compute_control(cur_odom, prev_odom), dtype=float)

    state_poses = _state_pose_table(loc_obj)
    prev_bel_flat = loc_obj.bel.reshape(-1)
    bel_bar_flat = np.zeros_like(prev_bel_flat, dtype=float)

    active_indices = np.flatnonzero(prev_bel_flat > _ACTIVE_BELIEF_THRESHOLD)
    if active_indices.size == 0:
        active_indices = np.array([int(np.argmax(prev_bel_flat))], dtype=int)

    for prev_idx in active_indices:
        prev_pose = state_poses[prev_idx]
        rot1_hat, trans_hat, rot2_hat = _compute_control_vectorized(state_poses, prev_pose)

        transition_prob = (
            loc_obj.gaussian(_normalize_angle(rot1_hat - actual_u[0]), 0.0, loc_obj.odom_rot_sigma)
            * loc_obj.gaussian(trans_hat - actual_u[1], 0.0, loc_obj.odom_trans_sigma)
            * loc_obj.gaussian(_normalize_angle(rot2_hat - actual_u[2]), 0.0, loc_obj.odom_rot_sigma)
        )
        bel_bar_flat += prev_bel_flat[prev_idx] * transition_prob

    total = np.sum(bel_bar_flat)
    if total > 0.0:
        bel_bar_flat /= total

    loc_obj.bel_bar = bel_bar_flat.reshape(loc_obj.bel.shape)
    return loc_obj.bel_bar


def sensor_model(obs, loc_obj=None):
    """Return per-beam Gaussian likelihoods for one candidate observation."""
    loc_obj = _require_loc(loc_obj)
    if loc_obj.obs_range_data is None:
        raise RuntimeError("No observation data available. Call loc.get_observation_data() first.")

    expected_obs = np.asarray(obs, dtype=float).reshape(-1)
    measured_obs = np.asarray(loc_obj.obs_range_data, dtype=float).reshape(-1)

    if expected_obs.shape != measured_obs.shape:
        raise ValueError("Expected observation and measured observation shapes do not match.")

    return loc_obj.gaussian(measured_obs, expected_obs, loc_obj.sensor_sigma)


def update_step(loc_obj=None):
    """Update loc.bel from loc.bel_bar using the latest observation data."""
    loc_obj = _require_loc(loc_obj)
    if loc_obj.obs_range_data is None:
        raise RuntimeError("No observation data available. Call loc.get_observation_data() first.")

    measured_obs = np.asarray(loc_obj.obs_range_data, dtype=float).reshape(-1)
    expected_obs = loc_obj.mapper.obs_views[..., : measured_obs.size]

    residual = measured_obs.reshape((1, 1, 1, -1)) - expected_obs

    # Use log-likelihoods for numerical stability; the common Gaussian constant
    # cancels out after normalization.
    log_likelihood = -0.5 * np.sum((residual / loc_obj.sensor_sigma) ** 2, axis=-1)
    log_likelihood -= np.max(log_likelihood)

    measurement_prob = np.exp(log_likelihood)
    bel = loc_obj.bel_bar * measurement_prob

    total = np.sum(bel)
    if total <= 0.0:
        bel = measurement_prob
        total = np.sum(bel)

    loc_obj.bel = bel / total
    return loc_obj.bel


__all__ = [
    "bind_context",
    "compute_control",
    "odom_motion_model",
    "prediction_step",
    "sensor_model",
    "update_step",
]
