"""
run_aloha_playback.py

Replays recorded actions in a real-world ALOHA environment using action trunking.
"""

import logging
import os
import sys
import time
import h5py
import zmq
import numpy as np
import draccus
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Union, Dict

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger(__name__)


@dataclass
class PlayConfig:
    # fmt: off

    #################################################################################################################
    # parameters
    #################################################################################################################
    hdf5_path: Union[str, Path] = ""                 # Path to HDF5 file with recorded data
    use_relative_actions: bool = False               # Whether to use relative actions (delta joint angles)
    trunk_size: int = 25                             # Number of actions to execute in each trunk
    loop_playback: bool = False                      # Whether to loop playback continuously
    playback_speed: float = 1.0                      # Playback speed multiplier (1.0 = realtime)

    #################################################################################################################
    # environment-specific parameters
    #################################################################################################################
    max_steps: int = 1000                            # Max number of steps per playback

    #################################################################################################################
    # Utils
    #################################################################################################################
    run_id_note: Optional[str] = None                # Extra note to add to end of run ID for logging
    local_log_dir: str = "./logs"                    # Local directory for playback logs
    save_video: bool = False                         # Save video of playback
    video_fps: int = 10                              # FPS for saved video

    seed: int = 7                                    # Random Seed (for reproducibility)

    # fmt: on


def validate_config(cfg: PlayConfig) -> None:
    """Validate configuration parameters."""
    if not cfg.hdf5_path:
        raise ValueError("HDF5 path must be provided for playback")
    if not Path(cfg.hdf5_path).exists():
        raise FileNotFoundError(f"HDF5 file not found at {cfg.hdf5_path}")
    if cfg.trunk_size <= 0:
        raise ValueError("Trunk size must be greater than 0")


def setup_logging(cfg: PlayConfig):
    """Set up logging to file."""
    # Create run ID
    run_id = f"PLAYBACK-{Path(cfg.hdf5_path).stem}"
    if cfg.run_id_note is not None:
        run_id += f"--{cfg.run_id_note}"

    # Set up local logging
    os.makedirs(cfg.local_log_dir, exist_ok=True)
    local_log_filepath = os.path.join(cfg.local_log_dir, run_id + ".txt")
    log_file = open(local_log_filepath, "w")
    logger.info(f"Logging to local log file: {local_log_filepath}")

    return log_file, local_log_filepath, run_id


def log_message(message: str, log_file=None):
    """Log a message to console and optionally to a log file."""
    print(message)
    logger.info(message)
    if log_file:
        log_file.write(message + "\n")
        log_file.flush()


def load_recorded_data(hdf5_path: Union[str, Path]):
    """Load recorded data from HDF5 file."""
    with h5py.File(hdf5_path, 'r') as f:
        data = {
            'actions': np.array(f['action']),
            'qpos': np.array(f['observations/qpos']),
            'images': {
                'cam_left_wrist': np.array(f['observations/images/cam_left_wrist']),
                'cam_right_wrist': np.array(f['observations/images/cam_right_wrist']),
                'cam_high': np.array(f['observations/images/cam_high']),
            }
        }
        # Check if we have enough data
        num_steps = data['actions'].shape[0]
        logger.info(f"Loaded {num_steps} recorded steps from {hdf5_path}")
        return data, num_steps
    
def run_play(
    cfg: PlayConfig,
    recorded_data: Dict,
    num_recorded_steps: int,
    log_file=None,
):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    machine_ip = "192.168.2.147"
    socket.connect(f"tcp://{machine_ip}:5555")
    socket.RCVTIMEO = 60000 # timeout: > 10s

    actions = recorded_data['actions']
    
    try:
        try:
            socket.send_pyobj({
                'command': 'RESET'
            })
            log_message('Robot Reset Request Sent', log_file=log_file)
            isreset = socket.recv_string()
            if isreset == 'Reset Done.':
                log_message("Robot Reset Done", log_file=log_file)
            else:
                log_message(f"Error reset robot", log_file=log_file)
                return False
        except zmq.error.Again:
                logger.error("Timeout waiting for observations")
                return False
        except Exception as e:
                logger.error(f"Error reset robot: {e}")
                return False

        num_trunks = (num_recorded_steps + cfg.trunk_size - 1) // cfg.trunk_size
        
        for trunk_idx in range(num_trunks):
            # 1. Obsercation Obtaining
            try:
                socket.send_pyobj({
                    'command':'GET_OBS'
                })
                log_message('Observation Requst Sent', log_file=log_file)
                observations = socket.recv_pyobj()
                if observations:
                    log_message(f"Received observations from robot: {observations.keys()}", log_file=log_file)
                else:
                    log_message(f"Invalid observations received from robot: {observations}", log_file=log_file)
                    break
            except zmq.error.Again:
                logger.error("Timeout waiting for observations")
                break
            except Exception as e:
                logger.error(f"Error receiving observations: {e}")
                break

            # TODO: 2. Model Inference
            time.sleep(1.0/cfg.playback_speed)

            # 3. Actions Execution (trunk)
            start_idx = trunk_idx * cfg.trunk_size
            end_idx = min(start_idx + cfg.trunk_size, num_recorded_steps)
            trunk_actions = actions[start_idx:end_idx]

            try:
                socket.send_pyobj({
                    'command': 'EXECUTE_ACTIONS',
                    'actions': trunk_actions.tolist()
                })

                response = socket.recv_string()
                if response == "ACK":
                    log_message(f"Trunk {trunk_idx+1}/{num_trunks} executed", log_file=log_file)
                else:
                    logger.error(f"Error from robot: {response}")
                    break
            except zmq.error.Again:
                logger.error("Timeout waiting for robot response (after actions)")
                break
            except Exception as e:
                logger.error(f"Error sending actions: {e}")
                break

            time.sleep(1.0/cfg.playback_speed)
    
    finally:
        socket.close()
        context.term()

@draccus.wrap()
def play_r1lite(cfg: PlayConfig) -> None:
    """Main function to playback recorded actions using action trunking."""
    # Validate configuration
    validate_config(cfg)
    
    # Setup logging
    log_file, local_log_filepath, run_id = setup_logging(cfg)
    
    # Load recorded data
    recorded_data, num_recorded_steps = load_recorded_data(cfg.hdf5_path)
    
    # Start playback
    log_message(f"\nStarting playback of {num_recorded_steps} recorded steps", log_file)
    log_message(f"Trunk size: {cfg.trunk_size}", log_file)
    log_message(f"Using {'relative' if cfg.use_relative_actions else 'absolute'} actions", log_file)
    log_message(f"Loop playback: {'ON' if cfg.loop_playback else 'OFF'}", log_file)
    log_message(f"Playback speed: {cfg.playback_speed}x", log_file)
    
    run_play(
        cfg, recorded_data, num_recorded_steps, log_file
    )
    
    # Close log file
    if log_file:
        log_file.close()


if __name__ == "__main__":
    time.sleep(1) # wait for the preparation of observation
    play_r1lite()

