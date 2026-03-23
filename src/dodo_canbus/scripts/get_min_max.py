import json
from collections import defaultdict

# 存储每个电机的最小/最大位置
min_pos = defaultdict(lambda: float('inf'))
max_pos = defaultdict(lambda: float('-inf'))

file_path = "stand.jsonl"

with open(file_path, "r") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue

        data = json.loads(line)

        names = data["name"]
        positions = data["position"]

        for name, pos in zip(names, positions):
            if pos < min_pos[name]:
                min_pos[name] = pos
            if pos > max_pos[name]:
                max_pos[name] = pos

# 输出结果
print("=== Joint Limits ===")
for key in sorted(min_pos.keys()):
    print(f"{key}: q_min={min_pos[key]:.6f}, q_max={max_pos[key]:.6f}")

print("\n=== ROS2 PARAM FORMAT ===")

keys = []
q_mins = []
q_maxs = []

for key in sorted(min_pos.keys()):
    keys.append(key)
    q_mins.append(min_pos[key])
    q_maxs.append(max_pos[key])

print("joint_limit_keys:", keys)
print("joint_q_mins:", [round(x, 6) for x in q_mins])
print("joint_q_maxs:", [round(x, 6) for x in q_maxs])