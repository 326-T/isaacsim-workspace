from typing import List

import numpy as np
from omni.isaac.core.prims.xform_prim import XFormPrim

from .factory import create_convex_rigid_body

UPPER_SHELF_HEIGHT: float = 1.18
MIDDLE_SHELF_HEIGHT: float = 0.635
SHELF_CENTER_X: float = 0.70


class ConvenienceStore:
    """
    コンビニエンスストア環境を再現したクラス
    """

    def __init__(
        self,
        num_product_rows: int = 1,
        gap: int = 0,
        num_product_columns: int = 1,
        store_usd_path: str = "isaacsim_ext/static/usd/tana_zentai.usd",
        goods_usd_path: str = "isaacsim_ext/static/usd/java_curry_chukara/java_curry_chukara.usd",
    ) -> None:
        """
        Initialize ConvenienceStore

        Args:
            num_product_rows (int, optional):
                棚に並べる商品の行数.
                Defaults to 1.
            num_product_columns (int, optional):
                棚に並べる商品の列数.
                Defaults to 1.
            store_usd_path (str, optional):
                コンビニエンスストア棚のUSDファイルのパス.
                Defaults to "isaacsim_ext/static/usd/tana_zentai.usd".
            goods_usd_path (str, optional):
                商品のUSDファイルのパス.
                Defaults to "isaacsim_ext/static/usd/java_curry_chukara/java_curry_chukara.usd".
        """
        self._shelf: XFormPrim = create_convex_rigid_body(
            name="shelf",
            prim_path="/World/convenience_store/tana_zentai",
            usd_path=store_usd_path,
            mass=100.0,
            static_friction=0.4,
            dynamic_friction=0.3,
            position=np.array([0.49292, 1.49365, 0.72202]),
            orientation=np.array([0.5, 0.5, 0.5, 0.5]),
        )

        self._products: List = []
        for i in range(num_product_columns):
            for j in range(num_product_rows):
                self._products.append(
                    create_convex_rigid_body(
                        name="java_curry_chukara{}_{}".format(i, j),
                        prim_path="/World/convenience_store/java_curry_chukara_{}_{}".format(
                            i, j
                        ),
                        usd_path=goods_usd_path,
                        mass=0.104,
                        static_friction=0.3,
                        dynamic_friction=0.2,
                        position=np.array(
                            object=[
                                SHELF_CENTER_X,
                                i * (gap + 1) * 0.08,
                                UPPER_SHELF_HEIGHT + j * 0.03,
                            ]
                        ),
                    )
                )

    @property
    def products(self) -> List[XFormPrim]:
        """
        商品のリスト

        Returns:
            List[XFormPrim]: 商品のリスト
        """
        return self._products

    @property
    def shelf(self) -> XFormPrim:
        """
        棚

        Returns:
            XFormPrim: 棚
        """
        return self._shelf
