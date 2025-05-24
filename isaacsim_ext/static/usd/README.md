# シミュレーション用の設定ファイル

## 各ファイルの説明

| ファイル名                | 内容                                                                                                                                                                                                                                                                                                                  |
| ------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ur5e.usd                  | `omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/`をコピーしたもの                                                                                                                                                                                                                                    |
| robotiq_2f140.usd         | `omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/Robotiq/2F-140/_f140_instanceable`を[https://medium.com/@joolab/how-to-set-up-closed-loop-gripper-with-robotiq-2f-85-f7aac12936b2](https://medium.com/@joolab/how-to-set-up-closed-loop-gripper-with-robotiq-2f-85-f7aac12936b2)を参考に加工したもの |
| ur5e_with_gripper.usd     | `ur5e.usd`と`robotiq_2f140.usd`を[https://medium.com/@joolab/how-to-assemble-a-gripper-on-a-robot-manipulator-in-isaac-sim-f7ba8e6f18fc](https://medium.com/@joolab/how-to-assemble-a-gripper-on-a-robot-manipulator-in-isaac-sim-f7ba8e6f18fc})を参考にアセンブルしたもの                                            |
| ur5e_with_2f140_scene.usd | 上記の作業に使用したステージ                                                                                                                                                                                                                                                                                          |

## プリムパス

- ur5e_with_gripper
  - ur5e
    - base_link
      - root_joint
    - base_link_inertia
      - shoulder_pan_joint
    - shoulder_link
      - shoulder_lift_joint
    - upper_arm_link
      - elbow_joint
    - forearm_link
      - wrist_1_joint
    - wrist_1_link
      - wrist_2_joint
    - wrist_2_link
      - wrist_3_joint
    - wrist_3_link
      - wrist_3_flange
    - flange
  - robotiq_2f140
    - robotiq_arg2f_base_link
      - finger_joint
      - left_inner_knuckle_joint
      - right_inner_knuckle_joint
      - right_outer_knuckle_joint
    - left_outer_knuckle
      - left_outer_finger_joint
    - left_outer_finger
      - left_inner_finger_joint
    - left_inner_finger
      - left_inner_finger_pad_joint
      - left_inner_knucle_joint
    - left_inner_finger_pad
    - left_inner_knuckle
    - right_outer_knuckle
      - right_outer_finger_joint
    - right_outer_finger
      - right_inner_finger_joint
    - right_inner_finger
      - right_inner_finger_pad_joint
      - right_inner_knucle_joint
    - right_inner_finger_pad
    - right_inner_knuckle

## 動くジョイント

- ur5e
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
- robotiq_2f140
  - finger_joint
  - right_outer_knuckle_joint

---

## UR5e + Robotiq 2F-140 の準備

### USD ファイルの取得

Omniverse で以下のコンテンツをダウンロードする

- ISAAC SIM ASSETS PACK 1
- ISAAC SIM ASSETS PACK 2
- ISAAC SIM ASSETS PACK 3

Isaac Sim 上で以下をインポートできるようになる

- UR5e: omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd
- 2F-140: omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/Robotiq/2F-140/\_f140_instanceable

### Robotiq 2F-140 の組み立て

`Robotiq 2F-140`をそのままだと使えず組み立て作業をする必要がある

1. ジョイントの作成
2. 特定のジョイントをアーティキュレーションからの除外
3. 特定の関節の角度制限を外す
4. 動かすジョイントに適切な`stiffness`と`damping`を設定する

上から 3 つは[こちらのブログ](https://medium.com/@joolab/how-to-set-up-closed-loop-gripper-with-robotiq-2f-85-f7aac12936b2)参照<br>
`stiffness`と`damping`の設定値は[こちらのフォーラム](https://forums.developer.nvidia.com/t/robotiq-2f-140-gripper-not-moves-smoothly/241932)参照

適切に組み立てられていると以下が確認できる

- 再生時に外力を加えると動く(1,2,3)
- target position を指定すると追従する(4)

| 項目      | 設定値 |
| --------- | ------ |
| stiffness | 200    |
| damping   | 20     |

### UR5e と Robotiq2F-140 を結合する

[こちらのブログ](https://medium.com/@joolab/how-to-assemble-a-gripper-on-a-robot-manipulator-in-isaac-sim-f7ba8e6f18fc)参照

適切にくっつけられてると、`再生`ボタンを押した後にアーティキュレーションを確認するとジョイントがまとまって表示される。
