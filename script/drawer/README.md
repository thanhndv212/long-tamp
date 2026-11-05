# Drawer open/close

A Panda opens a cabinet's drawer by its handle, lets go, grasps it again and
pushes it shut: the articulated-object example of #161.

```bash
python script/drawer/task_drawer.py               # then replays in the viewer
python script/drawer/task_drawer.py --no-viewer --open 0.12
```

The cabinet is declared `root: anchor` in `config/drawer_config.yaml`. It is
fixed at its initial pose, and its one prismatic joint, `cabinet/drawer`, is
its configuration. The SRDF gives it a handle but no contact surface, so the
constraint-graph factory locks the drawer whenever the handle is free and
frees it while the gripper holds the handle. Each slide is a `plan_loop` in
the grasp state, to a target where the drawer joint has moved and the arm
follows by IK.

## Assets

| File | Notes |
|---|---|
| `assets/cabinet.urdf` | authored here. A hollow carcass and one drawer near the top, with its handle bar at z = 0.6. Lower, a Panda reaching straight forward runs its joint 6 into the limit. |
| `assets/cabinet.srdf` | authored here. The handle, approached 0.4 rad from above about the bar's axis. |
| Panda, ground | `script/twin/assets` (see its README) |
