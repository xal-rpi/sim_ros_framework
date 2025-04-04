"""
Utility functions for creating checkpoints and restoring from them.
"""

import os
from typing import Dict, Any, Mapping

import collections
import yaml

import numpy as np
from flax.training import orbax_utils
import orbax.checkpoint as ckpts

try :
    from torch.utils.tensorboard import SummaryWriter as TBSummaryWriter
    TB_AVAILABLE = True
except ImportError:
    TB_AVAILABLE = False

try:
    import tensorflow
    TF_AVAILABLE = True
    class TFSummaryWriter:
        """ A wrapper for the tensorflow summary writer
        """
        def __init__(self, log_dir):
            self.writer = \
                tensorflow.summary.create_file_writer(logdir=log_dir)

        def add_scalar(self, tag, value, step):
            """ Add a scalar to the tensorboard
            """
            with self.writer.as_default():
                tensorflow.summary.scalar(tag, value, step=step)
except ImportError:
    TF_AVAILABLE = False

try :
    from tensorboardX import SummaryWriter as TBXSummaryWriter
    TBX_AVAILABLE = True
except ImportError:
    TBX_AVAILABLE = False

writer_available = TB_AVAILABLE or TF_AVAILABLE or TBX_AVAILABLE
if not (TB_AVAILABLE or TF_AVAILABLE or TBX_AVAILABLE):
    print("No logging possible: Tensorboard not available...")
    print("No package found to write events to Tensorboard.")
    print("Set agent's `write_interval` setting to 0 to disable writing")
    print("or install one of the following packages:")
    print("  - PyTorch: https://pytorch.org/get-started/locally")
    print("  - TensorFlow: https://www.tensorflow.org/install")
    print("  - TensorboardX: https://github.com/lanpa/tensorboardX#install")
    print("The current running process will be terminated.")
    # raise RuntimeError("No package for tensorboard logging found.")
else: # Use the first available
    if TB_AVAILABLE:
        SummaryWriter = TBSummaryWriter
    elif TF_AVAILABLE:
        SummaryWriter = TFSummaryWriter
    else:
        SummaryWriter = TBXSummaryWriter


def full_path(path : str) -> str:
    """ Complete the path with the user home directory if needed

    Args:
        path (str): The path to complete

    Returns:
        str: The completed path
    """
    return os.path.expanduser(path)

def load_yaml(config_path : str) -> Dict[str, Any]:
    """Load a yaml file from a given path

    Args:
        config_path: The path to the yaml file
            str

    Returns:
        yaml_dict: The dictionary containing the configuration in the yaml file
            dict
    """
    yml_file = open(full_path(config_path), 'r', encoding="utf-8")
    yml_byte = yml_file.read()
    cfg_train = yaml.load(yml_byte, yaml.SafeLoader)
    yml_file.close()
    return cfg_train

def dump_yaml(
    config_path : str,
    dict_val : Dict[str, Any]
):
    """Dump a dictionary to a yaml file

    Args:
        config_path: The path to the yaml file
            str
        dict_val: The dictionary to dump
            dict
    """
    with open(config_path, 'w', encoding="utf-8") as f:
        yaml.dump(dict_val, f)

def convert_dict_of_array_to_dict_of_list(
    config : Dict[str, Any]
) -> Dict[str, Any]:
    """Convert a dictionary of arrays to a dictionary of lists

    Args:
        config: The configuration dictionary
            dict

    Returns:
        dict_list: The configuration dictionary with all the arrays 
        converted to lists
            dict
    """
    if not isinstance(config, Mapping):
        return config

    new_config = {}
    for key, value in config.items():
        if isinstance(value, Mapping):
            new_config[key] = \
                convert_dict_of_array_to_dict_of_list(value)
        elif isinstance(value, list):
            new_config[key] = \
                [convert_dict_of_array_to_dict_of_list(v) for v in value]
        elif isinstance(value, tuple):
            new_config[key] = \
                tuple([convert_dict_of_array_to_dict_of_list(v) for v in value])
        else:
            new_config[key] = \
                value.tolist() if hasattr(value, 'tolist') else value
    return new_config

def load_saved_data_from_checkpoint(
    experiment_dir: str,
    checkpoint_name: str,
    best_mode: str = 'min',
    step : int = -1
) -> Dict[str, Any]:
    """
    Load the saved data from a checkpoint
    
    Args:
        experiment_dir: The directory of the experiment
            str
        checkpoint_name: The name of the checkpoint
            str
        step: The step of the checkpoint to restore. -1 means the latest,
        -2 means the best, -3 means the second best and so on, and any other
        non-negative integer means the corresponding step of the checkpoint
            int
    """
    # Construct the checkpoint manager
    ckpt_manager = TrainCheckpoints(
        experiment_dir,
        checkpoint_name,
        {},
        best_mode = best_mode,
        writer_on = False
    )
    # Restore the data
    config, path_data = ckpt_manager.restore_checkpoint(step)
    return config, path_data


class TrainCheckpoints:
    """
    A class for managing the checkpoints and the tensorboard writer
    """
    def __init__(
        self,
        experiment_dir: str,
        output_name: str,
        ckpt_cfg: Dict[str, Any] = {},
        best_mode = 'min',
        writer_on: bool = True,
        extra_config_to_save_as_yaml: dict = {},
        saving_freq: int = 1,
    ):
        """
        Initialize the checkpoint manager and the tensorboard writer for logging
        training and testing progress.

        Args:
            experiment_dir: The directory folder that will contain the checkpoints
                and the tensorboard logs.
                str
            ckpt_cfg: The configuration for the checkpoints.
                dict
            best_mode: The mode for the best checkpoint
            output_name: The name of the output file/folder
                str
            writer_on: Whether this code is use to write or read
                bool
            extra_config_to_save_as_yaml: Extra configuration to save as yaml in
            experiment_dir/output_name/config.yaml
                dict
            saving_freq: The frequency of saving the checkpoints
                int
        """
        # Counter for the number of updates
        self.counter_update = 0
        self.saving_freq = saving_freq
        self.best_mode = best_mode

        # Create the directory for the output if it does not exist
        self.experiment_dir = experiment_dir
        os.makedirs(self.experiment_dir, exist_ok=True)

        # Create the output directory
        self.output_dir = os.path.join(self.experiment_dir, output_name)
        # If the name already exists add a _ to the end
        while os.path.exists(self.output_dir) and writer_on:
            self.output_dir += "_"

        # Extract the writer if needed
        self.ckpt_dir = os.path.join(self.output_dir, "checkpoints")
        if writer_on:
            if not writer_available:
                raise RuntimeError("No package for tensorboard logging found.")
            self.writer = SummaryWriter(log_dir=self.output_dir)
            os.makedirs(self.ckpt_dir, exist_ok=True)

        # Save some temporary variables
        self.max_to_keep = ckpt_cfg.get('max_to_keep', None)
        self.metrics_cfg = ckpt_cfg.get('metrics', {})
        self.async_exec = ckpt_cfg.get('async_exec', False)
        self.timeout_async = ckpt_cfg.get('timeout_async', 60)

        # Let's save the metrics for best checkpoint
        metrics_file = os.path.join(self.ckpt_dir, 'metrics.yaml')
        config_file = os.path.join(self.output_dir, 'config.yaml')
        if writer_on:
            dump_yaml(metrics_file, self.metrics_cfg)
            if len(extra_config_to_save_as_yaml) > 0:
                # Convert it into a readable format
                extra_config_to_save_as_yaml = \
                    convert_dict_of_array_to_dict_of_list(
                    extra_config_to_save_as_yaml
                )
                dump_yaml(config_file, extra_config_to_save_as_yaml)
        else:
            self.metrics_cfg = load_yaml(metrics_file)
            # Load the extra configuration if exists
            if os.path.exists(config_file):
                self.extra_config = load_yaml(config_file)
            else:
                self.extra_config = {}

        # Buffer for storing tracking data
        self.tb_data = collections.defaultdict(list)

        # Log out the parameters of the checkpoints
        print(f'Checkpoints directory: {self.ckpt_dir}')
        print(f'Tensorboard and config directory: {self.output_dir}')
        print(f'Max to keep: {self.max_to_keep}')
        print(f'Async exec: {self.async_exec}')
        print(f'Timeout async: {self.timeout_async}')
        print(f'Saving Metrics:\n {self.metrics_cfg}')

        self.init_checkpoints_format()

    def init_checkpoints_format(self):
        """
        Initialize the checkpoints manager
        """
        def best_fn(x):
            """
            Temporary function to return the best value given
            the metric configuration
            """
            value = 0.0
            inexistant_key_default = \
                np.inf if self.best_mode == 'min' else -np.inf
            for k, v in self.metrics_cfg.items():
                value += x.get(k, inexistant_key_default) * v
            return value

        # Options for the checkpoint manager
        options = ckpts.CheckpointManagerOptions(
            save_interval_steps = 1,
            create = True,
            best_mode = self.best_mode,
            step_prefix = 'agent',
            best_fn = best_fn,
            max_to_keep = self.max_to_keep
        )
        if not self.async_exec:
            orbax_checkpointer = ckpts.PyTreeCheckpointer()
        else:
            orbax_checkpointer = ckpts.AsyncCheckpointer(
                ckpts.PyTreeCheckpointHandler(),
                timeout_secs = self.timeout_async
            )
        self.checkpoint_manager = ckpts.CheckpointManager(
            self.ckpt_dir,
            orbax_checkpointer,
            options
        )

    def should_update(self):
        """
        Check if it is time for updating or saving the checkpoint
        """
        should = self.counter_update % self.saving_freq == 0
        return should

    def terminate_async(self):
        """
        Terminate the async execution
        """
        if self.async_exec:
            self.checkpoint_manager.wait_until_finished()

    def track_data(self, data: Dict[str, np.ndarray]):
        """
        Store the datat that will be used for logging
        
        Args:
            data: The data to be stored
                dict
        """
        for k, v in data.items():
            self.tb_data[k].append(float(v))

    def write_data_to_tensorboard(self, step: int):
        """
        Write the data to the tensorboard
        
        Args:
            step: The step of the training
                int
        """
        for k, v in self.tb_data.items():
            self.writer.add_scalar(k, np.mean(v), step)
            # Add other data if needed
            #...
        self.tb_data.clear()

    def write_checkpoint(
        self,
        step: int,
        save_dict: Dict[str, Any],
        metrics_val: Dict[str, float]
    ):
        """ 
        Write the checkpoint and save the current configuration of the model

        Args:
            step: The step of the training
                int
            save_dict: The dictionary to be saved
                dict
            metrics_val: The metrics to use for the best checkpoint
                dict
        """
        save_args = orbax_utils.save_args_from_target(save_dict)
        self.checkpoint_manager.save(
            step,
            save_dict,
            save_kwargs = {'save_args': save_args},
            metrics = metrics_val
        )

    def write_checkpoint_and_log_data(
        self,
        save_dict: Dict[str, Any],
        metrics_val: Dict[str, float],
        step_factor: int = 1
    ):
        """
        Write the checkpoint and log the data to the tensorboard
        
        Args:
            save_dict: The dictionary to be saved
                dict
            metrics_val: The metrics to use for the best checkpoint
                dict
            step_factor: The factor to multiply the step for logging
                int
        """
        # TODO: Maybe something less constraining here for the type of
        # value to save through tensorboard
        metrics_val = { k : float(np.mean(v)) for k, v in metrics_val.items()}
        self.track_data(metrics_val)
        step = self.counter_update * step_factor
        if self.should_update():
            # print(save_dict)
            # print(metrics_val)
            self.write_checkpoint(step, save_dict, metrics_val)
            self.write_data_to_tensorboard(step)
        self.counter_update += 1

    def restore_checkpoint(self, step: int = -2) -> Dict[str, Any]:
        """ 
        Restore the checkpoint
        
        Args:
            step: The step of the checkpoint to restore. -1 means the latest,
            -2 means the best, -3 means the second best and so on, and any other
            non-negative integer means the corresponding step of the checkpoint
                int
        
        Returns:
            res_dict: The result dictionary
                dict
            ckpt_dir: The directory of the checkpoint
                str
        """

        # Restore the checkpoint
        if step == -1:
            step = self.checkpoint_manager.latest_step()
        elif step < -1:
            _, sorted_checkpoints = \
                self.checkpoint_manager._sort_checkpoints_by_metrics(
                    self.checkpoint_manager._checkpoints
                )
            step = sorted_checkpoints[step+1].step
        else:
            # step must be a non-negative integer
            assert step >= 0, 'The step must be a non-negative integer'
        # Restore the checkpoint
        res_dict = self.checkpoint_manager.restore(step)
        res_dict["saved_config"] = self.extra_config
        # Get the directory of the corresponding checkpoint
        ckpt_dir = os.path.join(self.ckpt_dir, f'agent_{step}')
        # Return the result dictionary
        return res_dict, ckpt_dir

    def get_latest_step(self):
        """
        Get the latest step of the checkpoint
        
        Returns:
            step: The latest step of the checkpoint
                int
        """
        step = self.checkpoint_manager.latest_step()
        if step is None:
            step = 0
        return step

    def get_best_step(self):
        """
        Get the best step of the checkpoint
        
        Returns:
            step: The best step of the checkpoint
                int
        """
        step = self.checkpoint_manager.best_step()
        if step is None:
            step = 0
        return step
