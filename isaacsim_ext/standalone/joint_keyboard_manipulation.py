from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False, "multi_gpu": False})

from omni.isaac.core import World
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.manipulators.grippers import ParallelGripper

from isaacsim_ext.callback import Logger, TimelineCallback
from isaacsim_ext.controller import JointController
from isaacsim_ext.task import ConveniPickupTask

my_world = World(stage_units_in_meters=1.0)

conveni_task = ConveniPickupTask()
my_world.add_task(task=conveni_task)
my_world.reset()

ur5e: Articulation = my_world.scene.get_object(
    conveni_task.get_params()["robot_name"]["value"]
)
gripper: ParallelGripper = conveni_task.get_params()["gripper"]["value"]
controller = JointController()
logger = Logger(
    world=my_world,
    robot_names=conveni_task.get_params()["robot_names"]["value"],
    x_form_prim_names=conveni_task.get_params()["x_form_prim_names"]["value"],
    contact_sensor_names=conveni_task.get_params()["contact_sensor_names"]["value"],
    camera_names=conveni_task.get_params()["camera_names"]["value"],
)
timeline_callback = TimelineCallback(
    on_start=logger.start,
    on_pause=logger.pause,
    on_stop=lambda: (logger.save_and_reset(), my_world.reset()),
)
logger.start()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        action: ArticulationAction = controller.forward(
            current_position=ur5e.get_joint_positions()
        )
        ur5e.get_articulation_controller().apply_action(action)

simulation_app.close()
