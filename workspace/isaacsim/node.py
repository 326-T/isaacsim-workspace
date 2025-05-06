# isaac_sim_loop.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Empty
import time


class IsaacSimLoop(Node):
    def __init__(self):
        super().__init__("isaac_sim_loop")
        # 最新の目標値を保存する変数
        self.latest_target = 0.0

        # サブスクライブ：controller からの目標値
        self.create_subscription(Float32, "controller/target", self.target_cb, 10)

        # パブリッシュ：状態とステップ完了通知
        self.pub_state = self.create_publisher(Float32, "sim/state", 10)
        self.pub_step = self.create_publisher(Empty, "sim/step", 10)

    def target_cb(self, msg: Float32):
        # コールバックで最新の目標値をキャッシュ
        self.latest_target = msg.data

    def run(self):
        while rclpy.ok():
            t0 = time.time()

            # ① メッセージ受信処理：最新の target_cb を呼び出す
            #    timeout_sec=0.0 なので、溜まっているコールバックだけを処理
            rclpy.spin_once(self, timeout_sec=0.0)

            # ② ダミーシミュレーション：latest_target を使って state を更新
            #    （ここでは単純に足し算／減算で擬似的に動かします）
            #    実際の IsaacSim API 呼び出しなら sim.step() 等をここで行う
            new_state = self.latest_target * 0.1  # ダミー

            # ③ 更新後の state を publish
            self.pub_state.publish(Float32(data=new_state))

            # ④ ステップ完了通知を publish
            self.pub_step.publish(Empty())

            self.get_logger().info(f"[sim] step → state={new_state:.2f}")


def main():
    rclpy.init()
    node = IsaacSimLoop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
