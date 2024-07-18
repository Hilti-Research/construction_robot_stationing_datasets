import gtsam.noiseModel
from include.measurement import read_dataset_from_yaml, StationingShape
import kinpy as kp
import numpy as np
import gtsam
from functools import partial
from matplotlib import pyplot as plt

# CONFIG
CALIBRATION_DATASET_PATH = "data/datasets/flat.yaml"
EVALUATION_DATASET_PATH = "data/datasets/pallet.yaml"
URDF_PATH = "data/trailblazer.urdf"


def skew_symmetric(v):
    return np.array([[0, -v.item(2), v.item(1)],
                     [v.item(2), 0, -v.item(0)],
                     [-v.item(1), v.item(0), 0]])


def hand_eye_position_factor(measurement, T_base_ee, idx_T_world_base, idx_t_ee_prism, this, values, jacobians):
    """Hand eye position factor
    residual = total_station_measurement - R_world_ee * t_ee_prism - t_world_ee
    with T_world_ee = T_world_base * T_base_ee

    :param measurement: Total station measurement, to be filled with `partial`
    :param T_base_ee: gtsam.Pose3, pose of end effector in base frame according to forward kinematics
    :param idx_T_world_base: index of T_world_base in values
    :param idx_t_ee_prism: index of t_ee_prism in values
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """
    # Read states
    T_world_base = values.atPose3(this.keys()[idx_T_world_base])
    t_ee_prism = values.atPoint3(this.keys()[idx_t_ee_prism])

    # Calculate T_base_ee and T_world_ee
    T_world_ee = T_world_base * T_base_ee

    # Calculate jacobians
    if jacobians is not None:
        H_left = np.zeros((3, 6))
        H_left[0:3, 0:3] = T_world_ee.rotation().matrix() * \
            skew_symmetric(t_ee_prism)
        H_left[0:3, 3:6] = -T_world_ee.rotation().matrix()

        # T_world_base
        jacobians[idx_T_world_base] = H_left @ T_base_ee.inverse().AdjointMap()

        # t_ee_prism
        jacobians[idx_t_ee_prism] = -T_world_ee.rotation().matrix()

    # Calculate error
    t_world_prism_est = T_world_ee.rotation().rotate(t_ee_prism) + \
        T_world_ee.translation()
    return measurement - t_world_prism_est


if __name__ == "__main__":
    # Load URDF
    robot = kp.build_chain_from_urdf(open(URDF_PATH).read())

    # Read in datasets
    _, calibration_measurements = read_dataset_from_yaml(
        CALIBRATION_DATASET_PATH, StationingShape.CROSS)
    evaluation_stations, _ = read_dataset_from_yaml(
        EVALUATION_DATASET_PATH, StationingShape.CROSS)

    # Prepare factor graph for calibration
    graph_calib = gtsam.NonlinearFactorGraph()
    # Pose of robot in world frame
    T_world_base_key = gtsam.symbol('T', 0)
    # Position of prism in end effector frame
    t_ee_prism_key = gtsam.symbol('P', 0)

    # Initial estimate
    initial_estimate = gtsam.Values()
    initial_estimate.insert(T_world_base_key, gtsam.Pose3())
    fk_res = robot.forward_kinematics({})
    T_root_ee = fk_res["dsr_link6"].matrix()
    T_root_prism = fk_res["prism"].matrix()
    T_ee_prism = np.linalg.inv(T_root_ee) @ T_root_prism
    t_ee_prism_init = gtsam.Point3(T_ee_prism[0:3, 3])
    initial_estimate.insert(t_ee_prism_key, t_ee_prism_init)

    # Add prior to end-effector prism pose
    prior = gtsam.noiseModel.Diagonal.Sigmas([0.05]*3)
    graph_calib.add(gtsam.PriorFactorPoint3(
        t_ee_prism_key, t_ee_prism_init, prior))

    # Add factors for calibration measurements
    position_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.001, 0.001, 0.001]))

    for measurement in calibration_measurements:
        # Forward kinematics for T_base_ee
        joint_config = {joint_name: measurement.joint_states[i] for i, joint_name in enumerate(
            measurement.joint_names)}
        fk_res = robot.forward_kinematics(joint_config)
        T_base_ee = fk_res["dsr_link6"].matrix()

        gf = gtsam.CustomFactor(position_noise, [T_world_base_key, t_ee_prism_key], partial(
            hand_eye_position_factor, measurement.t_plt_prism, gtsam.Pose3(T_base_ee), 0, 1))
        graph_calib.add(gf)

    # Optimize the factor graph
    params = gtsam.GaussNewtonParams()
    optimizer = gtsam.GaussNewtonOptimizer(
        graph_calib, initial_estimate, params)
    result = optimizer.optimize()

    # Extract calibration results
    t_ee_prism_est = result.atPoint3(t_ee_prism_key)
    print(f"Estimated t_ee_prism: {t_ee_prism_est}")

    # Perform evaluation using calibration results
    results = {}
    for station in evaluation_stations:
        # Prepare factor graph for evaluation
        graph_eval = gtsam.NonlinearFactorGraph()
        T_world_base_key = gtsam.symbol('T', 0)
        t_ee_prism_key = gtsam.symbol('P', 0)

        # Initial estimate
        initial_estimate = gtsam.Values()
        initial_estimate.insert(T_world_base_key, gtsam.Pose3())
        initial_estimate.insert(t_ee_prism_key, t_ee_prism_est)

        # Freeze end effector prism pose by adding hard prior
        prior = gtsam.noiseModel.Diagonal.Sigmas([1e-10]*3)
        graph_eval.add(gtsam.PriorFactorPoint3(
            t_ee_prism_key, t_ee_prism_est, prior))

        # Add factors for stationing measurements
        position_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.001, 0.001, 0.001]))
        for measurement in station.stationing_measurements:
            # Forward kinematics for T_base_ee
            joint_config = {joint_name: measurement.joint_states[i] for i, joint_name in enumerate(
                measurement.joint_names)}
            fk_res = robot.forward_kinematics(joint_config)
            T_base_ee = fk_res["dsr_link6"].matrix()

            gf = gtsam.CustomFactor(position_noise, [T_world_base_key, t_ee_prism_key], partial(
                hand_eye_position_factor, measurement.t_plt_prism, gtsam.Pose3(T_base_ee), 0, 1))
            graph_eval.add(gf)

        # Optimize the factor graph
        params = gtsam.GaussNewtonParams()
        optimizer = gtsam.GaussNewtonOptimizer(
            graph_eval, initial_estimate, params)
        result = optimizer.optimize()

        # Extract resulting T_world_base
        T_world_base_est = result.atPose3(T_world_base_key)

        # Calculate errors
        for evaluation_measurement in station.evaluation_measurements:
            # Forward kinematics for T_base_ee
            joint_config = {joint_name: evaluation_measurement.joint_states[i] for i, joint_name in enumerate(
                evaluation_measurement.joint_names)}
            fk_res = robot.forward_kinematics(joint_config)
            T_base_ee = fk_res["dsr_link6"].matrix()

            # Calculate predicted prism position in total station frame
            T_world_ee = T_world_base_est * gtsam.Pose3(T_base_ee)
            t_world_prism_est = T_world_ee.rotation().rotate(t_ee_prism_est) + \
                T_world_ee.translation()

            # Calculate error
            residual = evaluation_measurement.t_plt_prism - t_world_prism_est
            if station.height not in results:
                results[station.height] = []
            results[station.height].append(residual)

    # Calculate overall numerical 95% confidence interval in xy and z
    residuals = np.concatenate([np.array(res) for res in results.values()])
    r95_xy = np.percentile(np.linalg.norm(residuals[:, 0:2], axis=1), 95)
    r95_z = np.percentile(np.abs(residuals[:, 2]), 95)
    print(f"95% confidence interval in xy: {r95_xy*1e3:.1f} mm")
    print(f"95% confidence interval in z: {r95_z*1e3:.1f} mm")

    # Plot XY error distributions for all heights
    fig, axs = plt.subplots(1, len(results.keys())+1, figsize=(12, 4))

    heights = list(results.keys())
    for i in range(len(heights)):
        residuals_i = np.array(results[heights[i]])
        axs[i].hist(np.linalg.norm(residuals_i[:, 0:2] * 1e3, axis=1), bins=10, density=True)
        axs[i].set_title(f"Height: {heights[i]} m")
        axs[i].set_xlabel("XY Error [mm]")
        axs[i].set_ylabel("Density")
    axs[-1].hist(np.linalg.norm(residuals[:, 0:2] * 1e3, axis=1), bins=10, density=True)
    axs[-1].set_title("All heights")
    axs[-1].set_xlabel("XY Error [mm]")
    axs[-1].set_ylabel("Density")

    # Set same y and x limits for all subplots
    for ax in axs:
        ax.set_xlim([0, 30.0])
        ax.set_ylim([0, 0.2])
    # Turn off y-axis for all but the first subplot
    for ax in axs[1:]:
        ax.yaxis.set_visible(False)

    plt.show()
