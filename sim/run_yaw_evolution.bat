@echo off
cd /d "C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\sim"
"C:/Users/BillH/AppData/Local/Programs/ComfyUI/resources/uv/win/uv.exe" run --with "mujoco<3.12" --with numpy python nav_trials.py --panic --gens 10 > out\yaw_evo_panic.log 2>&1
"C:/Users/BillH/AppData/Local/Programs/ComfyUI/resources/uv/win/uv.exe" run --with "mujoco<3.12" --with numpy python nav_trials.py --race --gens 8 > out\yaw_evo_race.log 2>&1
