from operator import itemgetter

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False, "multi_gpu": False})
from omni.isaac.core import World
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.manipulators.grippers import ParallelGripper

from isaacsim_ext.callback import Logger, TimelineCallback
from isaacsim_ext.controller import DiffusionPolicyController, RMPFlowController
from isaacsim_ext.task import ConveniPickupTask

my_world: World = World(stage_units_in_meters=1.0)

conveni_task = ConveniPickupTask()
my_world.add_task(task=conveni_task)
my_world.reset()
ur5e: Articulation = my_world.scene.get_object(
    name=conveni_task.get_params()["robot_names"]["value"][0]
)
target: XFormPrim = my_world.scene.get_object(
    name=conveni_task.get_params()["x_form_prim_names"]["value"][0]
)
gripper: ParallelGripper = conveni_task.get_params()["gripper"]["value"]

rmpflow_controller = RMPFlowController(robot_articulation=ur5e)
target_controller = DiffusionPolicyController(
    checkpoint_path=".data/diffusion_policy_log/velocity_20250213_225124/checkpoints/10.ckpt",
    cameras=[
        my_world.scene.get_object(name=camera_name)
        for camera_name in conveni_task.get_params()["camera_names"]["value"]
    ],
)
logger = Logger(
    world=my_world,
    robot_names=conveni_task.get_params()["robot_names"]["value"],
    x_form_prim_names=conveni_task.get_params()["x_form_prim_names"]["value"],
    contact_sensor_names=conveni_task.get_params()["contact_sensor_names"]["value"],
    camera_names=conveni_task.get_params()["camera_names"]["value"],
    action_ref=lambda: target_controller.get_applied_action(),
    output_dir=".data/diffusion_policy_rollout",
)
timeline_callback = TimelineCallback(
    on_start=logger.start,
    on_pause=logger.pause,
    on_stop=lambda: (logger.save_and_reset(), my_world.reset()),
)

target_position, target_orientation = target.get_world_pose()
target.set_world_pose(
    position=target_position + np.array([0.1, 0, 0]),
    orientation=target_orientation,
)

logger.start()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        joint = ur5e.get_joint_positions()
        target_position, target_orientation = target.get_world_pose()

        next_position, next_orientation, optional_action = target_controller.forward(
            joint=joint, position=target_position, orientation=target_orientation
        )
        target.set_world_pose(
            position=next_position,
            orientation=next_orientation,
        )

        action: ArticulationAction = rmpflow_controller.forward(
            target_end_effector_position=next_position,
            target_end_effector_orientation=next_orientation,
        )
        ur5e.get_articulation_controller().apply_action(control_actions=action)

        if optional_action is not None:
            gripper_action: ArticulationAction = gripper.forward(optional_action)
            gripper.apply_action(
                ArticulationAction(
                    joint_positions=itemgetter(7, 9)(gripper_action.joint_positions)
                )
            )

simulation_app.close()
