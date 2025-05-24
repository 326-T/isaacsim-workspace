from typing import Any

import numpy as np
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.sensor import Camera, ContactSensor
from omni.kit.viewport.utility import create_viewport_window
from omni.kit.viewport.utility.legacy_viewport_window import LegacyViewportWindow
from omni.kit.viewport.window.window import ViewportWindow
from omni.physx.scripts import utils
from pxr import Sdf, Usd, UsdGeom, UsdPhysics
from pxr.Usd import Prim, Stage


def create_camera(
    name: str,
    prim_path: str,
    position: np.ndarray,
    orientation: np.ndarray,
    resolution: tuple[int, int] = (1280, 720),
) -> Camera:
    """
    Cameraインスタンスを生成する

    Args:
        name (str): カメラの名前. シーン内で一意である必要がある.
        prim_path (str): カメラのプリムパス.
        position (np.ndarray): カメラの位置.
        orientation (np.ndarray): カメラの姿勢.
        resolution (tuple[int, int], optional): 解像度. Defaults to (1280, 720).

    Returns:
        Camera: 生成されたカメラインスタンス
    """
    camera = Camera(
        name=name,
        prim_path=prim_path,
        position=position,
        frequency=20,
        resolution=resolution,
        orientation=euler_angles_to_quats(euler_angles=orientation, degrees=True),
    )
    camera.initialize()
    camera.set_focal_length(value=1.88)
    camera.set_focus_distance(value=40)
    camera.set_horizontal_aperture(value=2.7288)
    camera.set_vertical_aperture(value=1.5498)
    camera.set_clipping_range(near_distance=0.1, far_distance=5.0)
    return camera


def create_viewport(
    name: str,
    camera_path: str,
    position_x: int = 0,
    position_y: int = 0,
    width: int = 800,
    height: int = 400,
) -> ViewportWindow | LegacyViewportWindow | None:
    """
    ViewportWindowインスタンスを生成する

    Args:
        name (str): ViewportWindowの名前. シーン内で一意である必要がある.
        camera_path (str): カメラのプリムパス
        position_x (int, optional): ViewportWindowのX座標. Defaults to 0.
        position_y (int, optional): ViewportWindowのY座標. Defaults to 0.
        width (int, optional): ViewportWindowの幅. Defaults to 800.
        height (int, optional): ViewportWindowの高さ. Defaults to 400.

    Returns:
        ViewportWindow | LegacyViewportWindow | None: _description_
    """
    viewport: ViewportWindow | LegacyViewportWindow | None = create_viewport_window(
        name=name,
        camera_path=camera_path,
        position_x=position_x,
        position_y=position_y,
        width=width,
        height=height,
    )
    return viewport


def create_contact_sensor(name: str, link_path: str) -> ContactSensor:
    """
    ContactSensorインスタンスを生成する

    Args:
        name (str): ContactSensorの名前. シーン内で一意である必要がある.
        link_path (str): ContactSensorをアタッチするリンクのパス.

    Returns:
        ContactSensor: 生成されたContactSensorインスタンス
    """
    sensor = ContactSensor(
        prim_path=f"{link_path}/contact_sensor",
        name=name,
        min_threshold=0,
        max_threshold=10000000,
        radius=0.2,
        translation=np.array([0, 0, 0]),
    )
    return sensor


def create_light(
    name: str = "DistantLight",
    prim_path: str = "/World/Lights/DistantLight",
    position=np.array(object=[-1.2, 0, 4.0]),
    orientation=np.array(object=[0, -30, 0]),
) -> Prim | Any:
    """
    DistantLightインスタンスを生成する

    Args:
        name (str, optional): DistantLightの名前. シーン内で一意である必要がある. Defaults to "DistantLight".
        prim_path (str, optional): DistantLightのプリムパス. Defaults to "/World/Lights/DistantLight".
        position (_type_, optional): DistantLightの位置. Defaults to np.array(object=[-1.2, 0, 4.0]).
        orientation (_type_, optional): DistantLightの姿勢. Defaults to np.array(object=[0, -30, 0]).

    Returns:
        Prim | Any: 生成されたDistantLightインスタンス
    """
    stage: Stage | Any = get_current_stage()
    distant_light: Prim | Any = stage.DefinePrim(prim_path, name)
    distant_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(
        800.0
    )
    if not distant_light.HasAttribute("xformOp:translate"):
        UsdGeom.Xformable(distant_light).AddTranslateOp()
    distant_light.GetAttribute("xformOp:translate").Set(tuple(position.tolist()))
    if not distant_light.HasAttribute("xformOp:rotateXYZ"):
        UsdGeom.Xformable(distant_light).AddRotateXYZOp()
    distant_light.GetAttribute("xformOp:rotateXYZ").Set(tuple(orientation.tolist()))

    return distant_light


def create_convex_rigid_body(
    name: str,
    prim_path: str,
    usd_path: str,
    mass: float,
    static_friction: float = 0.3,
    dynamic_friction: float = 0.2,
    position=np.array(object=[0, 0, 0]),
    orientation=np.array(object=[0, 0, 0, 1]),
) -> XFormPrim:
    """
    convexDecompositionで近似した質量を持つ物体を生成する

    Args:
        name (str): 生成する物体の名前. シーン内で一意である必要がある.
        prim_path (str): 生成する物体のプリムパス.
        usd_path (str): USDファイルのパス.
        mass (float): 質量.
        static_friction (float, optional): 静止摩擦係数. Defaults to 0.3.
        dynamic_friction (float, optional): 動摩擦係数. Defaults to 0.2.
        position (_type_, optional): 位置. Defaults to np.array(object=[0, 0, 0]).
        orientation (_type_, optional): 姿勢. Defaults to np.array(object=[0, 0, 0, 1]).

    Returns:
        XFormPrim: 生成された物体インスタンス
    """
    add_reference_to_stage(
        usd_path=usd_path,
        prim_path=prim_path,
    )
    product = XFormPrim(
        prim_path=prim_path,
        name=name,
        translation=position,
        orientation=orientation,
    )
    prim: Prim | Any = get_prim_at_path(prim_path=prim_path)
    utils.setRigidBody(
        prim=prim,
        approximationShape="convexDecomposition",
        kinematic=False,
    )
    utils.addRigidBodyMaterial(
        stage=get_current_stage(),
        path=prim_path,
        staticFriction=static_friction,
        dynamicFriction=dynamic_friction,
    )

    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh):
            mass_api: None = UsdPhysics.MassAPI.Apply(p)
            mass_api.CreateMassAttr(mass)

    return product


def create_parallel_gripper(end_effector_prim_path: str) -> ParallelGripper:
    """
    ParallelGripperインスタンスを生成する

    Args:
        end_effector_prim_path (str): エンドエフェクタのプリムパス.

    Returns:
        ParallelGripper: 生成されたParallelGripperインスタンス
    """
    for joint in [
        "left_outer_knuckle_joint",
        "right_outer_knuckle_joint",
    ]:
        drive_apis = UsdPhysics.DriveAPI.GetAll(
            get_prim_at_path(prim_path=f"{end_effector_prim_path}/{joint}")
        )
        for drive_api in drive_apis:
            drive_api.GetMaxForceAttr().Set(100)
            drive_api.GetStiffnessAttr().Set(200)
            drive_api.GetDampingAttr().Set(200)

        revolute_joint = UsdPhysics.RevoluteJoint.Get(
            get_current_stage(), f"{end_effector_prim_path}/{joint}"
        )
        revolute_joint.GetLowerLimitAttr().Set(-42)
        revolute_joint.GetUpperLimitAttr().Set(42)

    return ParallelGripper(
        end_effector_prim_path=end_effector_prim_path,
        joint_prim_names=[
            "left_outer_knuckle_joint",
            "right_outer_knuckle_joint",
        ],
        joint_opened_positions=np.array(object=[0, 0]),
        # 41.53944 degrees is came from the UR5e_with_gripper.usd file.
        # If action_deltas is defined, this value is not used.
        joint_closed_positions=np.array(
            object=[41.53944 / 180 * np.pi, -41.53944 / 180 * np.pi]
        ),
        action_deltas=np.array([-0.1, 0.1]),
    )
