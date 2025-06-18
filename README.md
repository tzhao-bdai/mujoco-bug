# mujoco-bug 🐛
Interesting bug in mujoco 3.2.6

To run, first create a virtual environment with the correct version of mujoco, then run the bug script:
```
python3 -m venv env
source env/bin/activate
pip3 install -r requirements.txt
python3 bug.py
```
A visualizer will pop up, in which you can verify that the geometries are not in penetration, despite the negative signed distance function returned by `mj_geomDistance`.

