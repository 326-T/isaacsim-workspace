# スタンドアローンモード

isaac sim の起動無しで実行できる。Nucleaus の起動は必須。

各種クラスは[こちら参照](../isaacsim_ext/)。

## UR5e + Robotiq 2F140 をキーボードで操作する

```bash
$ python standalone/joint_keyboard_manipulation.py
```

| joint                     | up  | down |
| ------------------------- | --- | ---- |
| shoulder_pan_joint        | Q   | A    |
| shoulder_lift_joint       | W   | S    |
| elbow_joint               | E   | D    |
| wrist_1_joint             | R   | F    |
| wrist_2_joint             | T   | G    |
| wrist_3_joint             | Y   | H    |
| finger_joint              | U   | J    |
| right_outer_knuckle_joint | I   | K    |

## UR5e + Robotiq 2F140 を RMPflow で操作する

```bash
$ python standalone/rmpflow_keyboard_manipulation.py
```

| joint       | up  | down |
| ----------- | --- | ---- |
| transform X | W   | S    |
| transform Y | E   | D    |
| trasnform Z | R   | F    |
| rotate X    | U   | J    |
| rotate Y    | I   | K    |
| rotate Z    | O   | L    |
| gripper     | G   | H    |
