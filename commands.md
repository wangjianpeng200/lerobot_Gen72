# train 

```
DATA_DIR=data python lerobot/scripts/train.py     policy=act_koch_real     env=koch_real     dataset_repo_id=Wanacola/koch_pick_place_lego hydra.run.dir=outputs/train/act_koch_real --resume=true
```

# run

```
python lerobot/scripts/control_robot.py run_policy -p outputs/train/act_koch_real4/checkpoints/001000/pretrained_model/
```
# record

```
python lerobot/scripts/control_robot.py record_dataset     --fps 30     --root data     --repo-id Wanacola/koch_pick_place_lego6     --num-episodes 10     --run-compute-stats 1     --warmup-time-s 2     --episode-time-s 20 --reset-time-s 5
```

# replay

```
python lerobot/scripts/control_robot.py replay_episode     --fps 30     --root data     --repo-id Wanacola/koch_pick_place_lego  --episode 0
```
# visualize

```
python lerobot/scripts/visualize_dataset.py  --root data  --repo-id Wanacola/koch_pick_place_lego4  --episode-index 0
```




