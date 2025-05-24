from typing import Any, Callable

import omni.timeline


class TimelineCallback:
    """
    Timelineの再生、一時停止、停止ボタンを押した際に呼び出されるコールバック関数を設定するクラス
    """

    def __init__(
        self,
        on_start: Callable[[], None],
        on_pause: Callable[[], None],
        on_stop: Callable[[], None],
    ) -> None:
        """
        Timelineの再生、一時停止、停止ボタンを押した際に呼び出されるコールバック関数を設定するクラス

        Args:
            on_start (Callable[[], None]): 再生ボタンを押した際に呼び出されるコールバック関数
            on_pause (Callable[[], None]): 一時停止ボタンを押した際に呼び出されるコールバック関数
            on_stop (Callable[[], None]): 停止ボタンを押した際に呼び出されるコールバック関数
        """
        timeline: omni.timeline.Timeline = omni.timeline.get_timeline_interface()
        # サブスクリプションオブジェクトをインスタンス変数として保持
        self.subscription: (
            Any
        ) = timeline.get_timeline_event_stream().create_subscription_to_pop(
            self._on_timeline_event
        )
        self.on_start: Callable[[], None] = on_start
        self.on_pause: Callable[[], None] = on_pause
        self.on_stop: Callable[[], None] = on_stop

    def _on_timeline_event(self, event) -> None:
        """
        再生、一時停止、停止ボタンを押した際に呼び出されるコールバック関数

        Args:
            event (Any): TimelineEvent
        """
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            print("Simulation Started")
            self.on_start()
        elif event.type == int(omni.timeline.TimelineEventType.PAUSE):
            print("Simulation Paused")
            self.on_pause()
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            print("Simulation Stopped")
            self.on_stop()
