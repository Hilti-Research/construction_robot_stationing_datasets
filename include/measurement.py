import numpy as np
from typing import List, Dict, Any, Tuple
import yaml


class Measurement:
    """
    Class representing a measurement taken at a waypoint.

    Attributes:
        id (int): Measurement ID.
        time (float): Timestamp of measurement.
        upside_down (bool): Flag if end effector is upside down.
        doosan_cmd (np.ndarray): Cartesian command used for the manipulator.
        joint_names (List[str]): Names of the joints.
        joint_states (np.ndarray): Array of joint states.
        t_plt_prism (np.ndarray): Position measurement of the prism from total station.
        a_base (np.ndarray): Base accelerometer reading.
        a_column (np.ndarray): Column accelerometer reading.
    """

    def __init__(self, data_dict: Dict[str, Any], joint_names: List[str]):
        """
        Initializes the Measurement object with data from a dictionary and joint names.

        Args:
            data_dict (Dict[str, Any]): Dictionary containing measurement data.
            joint_names (List[str]): List of joint names.
        """
        self.id = data_dict['id']
        self.time = data_dict['time']
        self.upside_down = data_dict['upside_down']
        self.doosan_cmd = np.array(data_dict['doosan_cmd'])
        self.joint_names = joint_names
        self.joint_states = np.array(data_dict['joint_states'])
        self.t_plt_prism = np.array(data_dict['prism_position'])
        self.a_base = np.array(data_dict['tilt_sensor_base_accel'])
        self.a_column = np.array(data_dict['tilt_sensor_column_accel'])


class Station:
    """
    Class representing a station, meaning a set of stationing measurements and the
    corresponding evaluation measurements. Usually, whenever the lifting column was
    moved or the manipulator solution space was changed, a new station is created.

    Attributes:
        height (float): Station height.
        solution_space (int): Solution space of the manipulator.
        stationing_measurements (List[Measurement]): List of measurements taken at the station.
        evaluation_measurements (List[Measurement]): List of evaluation measurements taken at the station.
    """

    def __init__(self, height: float, solution_space: int):
        self.height = height
        self.solution_space = solution_space
        self.stationing_measurements = []
        self.evaluation_measurements = []


class StationingShape:
    SQUARE = 1
    CROSS = 2
    ALL = 3

    # List of cartesian coordinates defining which doosan_cmd points correspond
    # to the stationing point shapes.
    DOOSAN_CMD_SQUARE = [[0.1, 0.15, 0.48], [
        0.1, -0.15, 0.48], [0.4, 0.15, 0.48], [0.4, -0.15, 0.48]]
    DOOSAN_CMD_CROSS = [[0.25, 0.15, 0.48], [
        0.25, -0.15, 0.48], [0.4, 0.0, 0.48], [0.1, 0.0, 0.48]]


def read_dataset_from_yaml(file_path: str, stationing_points_grid: StationingShape = StationingShape.CROSS,
                           solution_spaces: List[int] = [0, 7]) -> Tuple[List[Station], List[Measurement]]:
    """
    Reads a dataset from a YAML file.

    Args:
        file_path (str): Path to the YAML file.
        stationing_points_grid (StationingShape): Shape of the stationing points grid.
        solution_spaces (List[int]): List of solution spaces to consider.

    Returns:
        Tuple[List[Station], List[Measurement]]: A tuple containing the list of Station objects and the merged list of all measurements.
        The list of stations only consists of the selected solution spaces, while the merged list contains all measurements in that dataset.
    """
    # Read in YAML file
    with open(file_path, 'r') as file:
        data = yaml.load(file, Loader=yaml.FullLoader)

    # Capture joint names
    joint_names = data['joint_names']

    # Define doosan_cmd point mask for corresponding stationing point shape
    if stationing_points_grid == StationingShape.SQUARE:
        doosan_cmd_mask = StationingShape.DOOSAN_CMD_SQUARE
    elif stationing_points_grid == StationingShape.CROSS:
        doosan_cmd_mask = StationingShape.DOOSAN_CMD_CROSS
    else:
        doosan_cmd_mask = StationingShape.DOOSAN_CMD_SQUARE + \
            StationingShape.DOOSAN_CMD_CROSS

    # Generate list of stations and merged list of all measurements
    stations = []
    merged_measurements = []

    for height in data['column_heights']:
        for solution_space in data['column_heights'][height]['solution_spaces']:
            station = Station(height, solution_space)
            for meas in data['column_heights'][height]['solution_spaces'][solution_space]['stationing_measurements']:
                if solution_space in solution_spaces and meas['doosan_cmd'] in doosan_cmd_mask and len(station.stationing_measurements) < len(doosan_cmd_mask):
                    station.stationing_measurements.append(
                        Measurement(meas, joint_names))
                merged_measurements.append(Measurement(meas, joint_names))
            for meas in data['column_heights'][height]['solution_spaces'][solution_space]['evaluation_measurements']:
                if solution_space in solution_spaces:
                    station.evaluation_measurements.append(
                        Measurement(meas, joint_names))
                merged_measurements.append(Measurement(meas, joint_names))
            stations.append(station)

    return stations, merged_measurements
