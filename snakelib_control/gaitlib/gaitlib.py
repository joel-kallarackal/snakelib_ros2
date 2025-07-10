import sys
from abc import ABCMeta, abstractmethod
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class Gaitlib(metaclass=ABCMeta):
    """Defines the abstract interface for gait libraries defined for different snakes.

    Subclasses of Gaitlib provide the implementations for each particular snake architecture.
    Subclasses are forced to provide implementations for all abstract methods and abstract
    properties.
    """

    def __init__(self, snake_type="REU"):
        self.create_gait()
        # Load YAML file
        params_path = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'snake_params.yaml')

        with open(params_path, "r") as file:
            data = yaml.safe_load(file)
        self._snake_param = data.get("command_manager").get("ros__parameters").get(f"{snake_type}", {})
        self.default_gait_params = self._snake_param.get("gait_params", {})

        self.current_gait_params = [0.0] * self.num_gait_param

        # String denoting the current gait being executed. Gets updated any time a call to a gait is made
        self.current_gait = None

    @abstractmethod
    def create_gait(self):
        pass

    # Define properties that subclasses are required to define
    @property
    @abstractmethod
    def snake_type(self):
        pass

    @property
    @abstractmethod
    def num_modules(self):
        pass

    @property
    @abstractmethod
    def num_gait_param(self):
        pass

    @property
    @abstractmethod
    def gait_params_filepath(self):
        """Filepath of the yaml file that stores the default gait parameters."""
        pass

    # Helper that updates parameters in a given dictionary of parameters
    def update_params(self, params_dict, params_to_update):
        z = params_dict.copy()
        z.update(params_to_update)
        return z

    # Parse parameters from a yaml file. These could be gait parameters or other configuration parameters.
    @staticmethod
    def parse_params_yaml(yaml_filepath):
        with open(yaml_filepath, "r") as ymlfile:
            return yaml.safe_load(ymlfile)
