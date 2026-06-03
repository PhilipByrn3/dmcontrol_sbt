"""
render.py
Visual inspection of the split-belt rimless wheel environment.

Two modes (set MODE below, or pass --mode on the command line):

    'frames'    — renders four camera angles as a static matplotlib grid and
                  saves a PNG to ../figures/. Good for a quick look without
                  needing a display-capable session.

    'viewer'    — launches the dm_control interactive 3-D viewer.
                  Click and drag to orbit; scroll to zoom; right-click to pan.

Usage:
    python render.py              # uses MODE constant below
    python render.py --mode frames
    python render.py --mode viewer
"""

import argparse
import sys
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from dm_control import mujoco, mjcf, viewer
from dm_control.rl.control import Environment

from sbt_core import SplitBeltSim, SplitBeltTask, load_config
from create_wheel import AssembleWheel
from create_treadmill import AssembleTreadmill

# ---------------------------------------------------------------------------
# Default mode — change to 'viewer' for interactive session
# ---------------------------------------------------------------------------
MODE = 'viewer'

FIGURES_PATH = Path(__file__).parent.parent / 'figures'

# ---------------------------------------------------------------------------
# Camera definitions
# Positions are in world coordinates; the axle sits at roughly (0, 0, 0.38).
# ---------------------------------------------------------------------------
CAMERAS = [
    # name, pos,               xyaxes (x-axis of image, y-axis of image)
    ('side',  [0.0, -1.5, 0.5],  [1, 0, 0,  0, 0.33, 1]),   # looking along +Y
    ('front', [1.5,  0.0, 0.5],  [0, 1, 0,  0, 0.33, 1]),   # looking along -X
    ('top',   [0.0,  0.0, 2.0],  [1, 0, 0,  0, 1,    0]),   # looking straight down
    ('iso',   [1.1, -1.1, 1.2],  [0.71, 0.71, 0, -0.24, 0.24, 0.94]), # isometric
]


def build_model_with_cameras(config: dict) -> mjcf.RootElement:
    """
    Build the full MuJoCo model and attach named inspection cameras.
    Reuses the same assembly logic as SplitBeltSim._build_model().
    """
    model = mjcf.RootElement(model='sbt_rimless_wheel')

    model.compiler.autolimits = True
    model.compiler.angle      = 'degree'

    model.option.gravity    = [0, 0, -9.81]
    model.option.timestep   = config['timestep']
    model.option.integrator = config['integrator']
    model.option.cone       = config['cone']
    model.option.solver     = config['solver']
    model.option.iterations = config['iterations']

    AssembleTreadmill(model.worldbody, config)
    AssembleWheel(model.worldbody, config)

    model.sensor.add('framepos', name='axlepos',
                     objtype='xbody', objname=AssembleWheel.BODY_NAME)

    # Add named cameras
    for name, pos, xyaxes in CAMERAS:
        model.worldbody.add('camera', name=name, pos=pos, xyaxes=xyaxes)

    return model


def warmup(physics, steps: int = 100):
    """
    Step the simulation briefly so the wheel has settled onto the belts.
    Without this, the first render shows the wheel floating at its initial height.
    """
    for _ in range(steps):
        physics.step()


def render_frames(config: dict,
                  height: int = 480,
                  width: int  = 640,
                  warmup_steps: int = 100,
                  save: bool = True):
    """
    Render one frame from each named camera and display as a 2×2 grid.

    Args:
        config:        simulation config dict
        height/width:  render resolution per panel
        warmup_steps:  physics steps before capturing frames
        save:          if True, save PNG to ../figures/

    Returns:
        fig, axes
    """
    model   = build_model_with_cameras(config)
    physics = mujoco.Physics.from_xml_string(model.to_xml_string())

    # Run a short warmup so the wheel rests on the belts
    warmup(physics, steps=warmup_steps)

    # Render one frame per camera
    frames = {}
    for name, _, _ in CAMERAS:
        frames[name] = physics.render(height=height, width=width, camera_id=name)

    # Display
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Split-Belt Rimless Wheel — Inspection Views', fontsize=14)

    for ax, (name, _, _) in zip(axes.flat, CAMERAS):
        ax.imshow(frames[name])
        ax.set_title(name.capitalize())
        ax.axis('off')

    plt.tight_layout()

    if save:
        FIGURES_PATH.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime('%Y-%m-%d__%H-%M-%S')
        path = FIGURES_PATH / f'render_{timestamp}.png'
        fig.savefig(path, dpi=150)
        print(f'Saved → {path}')

    plt.show()
    return fig, axes


def launch_interactive(config: dict):
    """
    Launch the dm_control interactive 3-D viewer.
    Controls: left-drag to orbit, scroll to zoom, right-drag to pan.
    """
    model   = build_model_with_cameras(config)
    physics = mujoco.Physics.from_xml_string(model.to_xml_string())
    task    = SplitBeltTask()
    env     = Environment(physics=physics, task=task)
    viewer.launch(env)


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Render the SBT rimless wheel.')
    parser.add_argument('--mode', choices=['frames', 'viewer'],
                        default=MODE, help='Render mode (default: %(default)s)')
    args = parser.parse_args()

    config = load_config()

    if args.mode == 'frames':
        render_frames(config)
    else:
        launch_interactive(config)


if __name__ == '__main__':
    main()
