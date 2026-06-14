from __future__ import annotations

import os
from pathlib import Path
import sys
import time

os.environ["CHESS_ROBOT_PYBULLET_GUI"] = "1"
os.environ.setdefault("CHESS_ROBOT_GUI_WIDTH", "1600")
os.environ.setdefault("CHESS_ROBOT_GUI_HEIGHT", "1000")

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import matplotlib.image as mpimg
import numpy as np

from src.common.config import DEFAULT_CONFIG
from src.simulation._runtime import RUNTIME, p
from src.simulation.load_robot import load_robot
from src.simulation.scene_builder import build_scene, set_human_safety_zone


def save_preview_image(output_path: Path, width: int = 1920, height: int = 1080) -> None:
    config = DEFAULT_CONFIG
    target = (
        config.board_origin[0] + (config.board_cols - 1) * config.cell_size / 2.0,
        config.board_origin[1] + (config.board_rows - 1) * config.cell_size / 2.0,
        config.z_board,
    )
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=target,
        distance=max(config.board_cols, config.board_rows) * config.cell_size * 1.6,
        yaw=42.0,
        pitch=-48.0,
        roll=0.0,
        upAxisIndex=2,
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=45.0,
        aspect=width / height,
        nearVal=0.01,
        farVal=5.0,
    )
    _, _, rgba, _, _ = p.getCameraImage(
        width,
        height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
        physicsClientId=RUNTIME.client_id,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image = np.reshape(rgba, (height, width, 4)).astype(np.uint8)
    mpimg.imsave(output_path, image)


def main() -> None:
    robot = load_robot()
    scene = build_scene(obstacle_mode="mode_3")
    set_human_safety_zone(True)
    for _ in range(60):
        p.stepSimulation(physicsClientId=RUNTIME.client_id)

    preview_path = PROJECT_ROOT / "results" / "simulation_preview.png"
    save_preview_image(preview_path)

    print("PyBullet GUI preview running.")
    print("Robot:", robot)
    print("Scene:", scene)
    print("Pieces:", RUNTIME.piece_body_ids)
    print("High-resolution preview image:", preview_path)
    print("Close this console or the PyBullet window when done.")

    while RUNTIME.client_id is not None and p.isConnected(RUNTIME.client_id):
        p.stepSimulation(physicsClientId=RUNTIME.client_id)
        time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    main()
