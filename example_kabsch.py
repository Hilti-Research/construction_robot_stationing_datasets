from include.measurement import read_dataset_from_yaml, StationingShape
import kinpy as kp
import numpy as np
from include.kabsch import kabsch
from matplotlib import pyplot as plt

# CONFIG
MEASUREMENT_PATH = "data/datasets/pallet.yaml"
URDF_PATH = "data/trailblazer.urdf"

if __name__ == "__main__":
    # Load URDF
    robot = kp.build_chain_from_urdf(open(URDF_PATH).read())

    # Read in stations
    stations, _ = read_dataset_from_yaml(MEASUREMENT_PATH, StationingShape.CROSS)

    # Predict prism positions of evaluation points using forward kinematics and compare
    # them against the actual total station measurements
    results = {}
    for station in stations:
        # Perform least-squares stationing using Kabsch
        # Get prism positions of stationing points as numpy array
        sp__t_plt_prism = np.array([measurement.t_plt_prism for measurement in station.stationing_measurements])

        # Calculate prism position in robot base frame from forward kinematics
        sp__t_base_prism = []
        for measurement in station.stationing_measurements:
            joint_config = {joint_name: measurement.joint_states[i] for i, joint_name in enumerate(measurement.joint_names)}

            # Perform forward kinematics
            fk_res = robot.forward_kinematics(joint_config)
            sp__t_base_prism.append(fk_res["prism"].matrix()[0:3, 3])
        sp__t_base_prism = np.array(sp__t_base_prism)

        # Perform Kabsch Least-squares stationing
        R_plt_base, t_plt_base, _ = kabsch(sp__t_plt_prism, sp__t_base_prism, scale=False)

        # For every evaluation point, calculate the predicted prism position in total station frame
        # given the estimated T_world_base and compare it against the actual total station measurement
        for measurement in station.evaluation_measurements:
            # Calculate predicted prism position in robot base frame
            joint_config = {joint_name: measurement.joint_states[i] for i, joint_name in enumerate(measurement.joint_names)}
            fk_res = robot.forward_kinematics(joint_config)
            t_base_prism = fk_res["prism"].matrix()[0:3, 3]

            # Calculate predicted prism position in total station frame
            est_t_plt_prism = R_plt_base @ t_base_prism + t_plt_base

            # Create results entry for height
            if station.height not in results:
                results[station.height] = []
            residual = measurement.t_plt_prism - est_t_plt_prism
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
