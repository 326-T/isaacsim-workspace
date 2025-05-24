from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path


class RMPFlowTarget(XFormPrim):
    """
    RMPFlowでエンドエフェクタの位置を指定するためのターゲットクラス

    Args:
        XFormPrim (_type_): XFormPrimクラスを継承
    """

    def __init__(self) -> None:
        add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd",
            prim_path="/World/target",
        )
        super().__init__(
            name="rmpflow_target",
            prim_path="/World/target",
            scale=[0.04, 0.04, 0.04],
        )
