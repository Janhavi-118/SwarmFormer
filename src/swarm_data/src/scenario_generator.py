import json
import itertools
import random
import math

# Random seed for reproducibility (optional)
random.seed(42)

# Parameters
formations = ["line", "circle", "wedge", "diamond", "grid"]  # 5 formations
spreads = ["tight", "spread", "random"]                      # 3 spreads
env_types = ["empty", "sparse", "medium", "dense"]           # 4 environments
robot_counts = [3, 5, 7, 11]                                 # 4 robot counts

# Environment to obstacle density and world file mapping
env_config_map = {
    "empty":  {"obstacle_density": 0.0,  "world_file": "empty_world.launch.py"},
    "sparse": {"obstacle_density": 0.15, "world_file": "sparse_world.launch.py"},
    "medium": {"obstacle_density": 0.30, "world_file": "medium_world.launch.py"},
    "dense":  {"obstacle_density": 0.45, "world_file": "dense_world.launch.py"}
}

ARENA_MIN, ARENA_MAX = -10, 10  # arena size in Gazebo

def random_point():
    """Returns a random (x, y) within arena bounds."""
    return random.uniform(ARENA_MIN, ARENA_MAX), random.uniform(ARENA_MIN, ARENA_MAX)

def generate_spawn_positions(formation, count, spread):
    """Generate formation-aware spawn positions inside arena with spread effects."""
    cx, cy = random_point()
    positions = []

    if spread == "tight":
        spacing = random.uniform(0.5, 1.0)
    elif spread == "spread":
        spacing = random.uniform(2.0, 4.0)
    else:  # random spread - fully random points per robot
        spacing = None

    if formation == "line":
        for i in range(count):
            if spacing is None:
                x, y = random_point()
            else:
                x = cx + (i - (count - 1) / 2) * spacing
                y = cy
            yaw = random.uniform(0, 2 * math.pi)
            positions.append([x, y, yaw])

    elif formation == "circle":
        radius = spacing if spacing else random.uniform(2.0, 5.0)
        angle_offset = random.uniform(0, 2 * math.pi)
        for i in range(count):
            angle = angle_offset + 2 * math.pi * i / count
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            yaw = angle + math.pi / 2
            positions.append([x, y, yaw])

    elif formation == "wedge":
        for i in range(count):
            if spacing is None:
                x, y = random_point()
            else:
                dir_mult = 1 if i % 2 == 0 else -1
                offset = (i // 2 + 1) * spacing
                x = cx + dir_mult * offset
                y = cy + offset
            yaw = random.uniform(0, 2 * math.pi)
            positions.append([x, y, yaw])

    elif formation == "diamond":
        rel_positions = [(0, -1), (-1, 0), (1, 0), (0, 1), (0, 0), (1, 1), (-1, 1)]
        factor = spacing if spacing else random.uniform(1.0, 3.0)
        for i in range(count):
            dx, dy = rel_positions[i % len(rel_positions)]
            x = cx + dx * factor
            y = cy + dy * factor
            yaw = random.uniform(0, 2 * math.pi)
            positions.append([x, y, yaw])

    elif formation == "grid":
        rows = math.ceil(math.sqrt(count))
        cols = math.ceil(count / rows)
        factor = spacing if spacing else random.uniform(1.0, 3.0)
        for i in range(count):
            r = i // cols
            c = i % cols
            x = cx + (c - cols / 2) * factor
            y = cy + (r - rows / 2) * factor
            yaw = random.uniform(0, 2 * math.pi)
            positions.append([x, y, yaw])

    # Clip positions inside arena boundaries
    for p in positions:
        p[0] = max(ARENA_MIN, min(ARENA_MAX, p[0]))
        p[1] = max(ARENA_MIN, min(ARENA_MAX, p[1]))
    return positions

def generate_formation_goals(formation, count):
    """Generate random formation-aligned goals inside arena."""
    gx, gy = random_point()
    goals = []
    angle_offset = random.uniform(0, 2 * math.pi)
    radius = random.uniform(2.0, 5.0)

    if formation == "line":
        spacing = random.uniform(1.0, 3.0)
        for i in range(count):
            x = gx + (i - (count - 1) / 2) * spacing
            y = gy
            goals.append([x, y])

    elif formation == "circle":
        for i in range(count):
            angle = angle_offset + 2 * math.pi * i / count
            x = gx + radius * math.cos(angle)
            y = gy + radius * math.sin(angle)
            goals.append([x, y])

    elif formation == "wedge":
        spacing = random.uniform(1.0, 3.0)
        for i in range(count):
            dir_mult = 1 if i % 2 == 0 else -1
            offset = (i // 2 + 1) * spacing
            x = gx + dir_mult * offset
            y = gy + offset
            goals.append([x, y])

    elif formation == "diamond":
        rel_positions = [(0, -1), (-1, 0), (1, 0), (0, 1), (0, 0), (1, 1), (-1, 1)]
        factor = random.uniform(1.0, 3.0)
        for i in range(count):
            dx, dy = rel_positions[i % len(rel_positions)]
            x = gx + dx * factor
            y = gy + dy * factor
            goals.append([x, y])

    elif formation == "grid":
        rows = math.ceil(math.sqrt(count))
        cols = math.ceil(count / rows)
        spacing = random.uniform(1.0, 3.0)
        for i in range(count):
            r = i // cols
            c = i % cols
            x = gx + (c - cols / 2) * spacing
            y = gy + (r - rows / 2) * spacing
            goals.append([x, y])

    # Clip goals inside arena boundaries
    for g in goals:
        g[0] = max(ARENA_MIN, min(ARENA_MAX, g[0]))
        g[1] = max(ARENA_MIN, min(ARENA_MAX, g[1]))
    return goals

# Generate all 240 unique scenario configs
combo_list = list(itertools.product(formations, spreads, env_types, robot_counts))

scenarios = []
scenario_id_counter = 0

# Generate 2 episodes per configuration for 480 episodes total
for formation, spread, env_type, swarm_size in combo_list:
    for _ in range(2):  # 2 episodes each
        env_cfg = env_config_map[env_type]
        spawns = generate_spawn_positions(formation, swarm_size, spread)
        goals = generate_formation_goals(formation, swarm_size)

        scenario = {
            "scenario_id": f"scenario_{scenario_id_counter:03d}",
            "swarm_size": swarm_size,
            "formation_type": formation,
            "environment_type": env_type,
            "initial_spread": spread,
            "obstacle_density": env_cfg["obstacle_density"],
            "world_file": env_cfg["world_file"],
            "spawn_positions": spawns,
            "formation_goals": goals
        }
        scenarios.append(scenario)
        scenario_id_counter += 1

# Save to JSON file
with open("diverse_scenarios.json", "w") as f:
    json.dump(scenarios, f, indent=2)

print(f"Generated {len(scenarios)} scenarios (expected 480)")
