from collections import defaultdict
from heapq import heappop, heappush
from itertools import combinations, count
from math import sqrt
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


class MapfSolutionGif:
    GRID_SIZE = 12
    CANVAS_SIZE = 720
    CELL_SIZE = 48
    GRID_ORIGIN = (72, 84)
    FRAMES_PER_MOVE = 4
    AGENT_OUTER_RADIUS = 13
    MAX_LOW_LEVEL_TIME = 80
    MAX_CBS_NODES = 50_000
    OUTPUT_PATH = Path(__file__).resolve().parents[1] / "assets" / "img" / "mapf-solution.gif"

    COLORS = {
        "background": "#0a202b",
        "panel": "#102d39",
        "grid": "#3b535e",
        "obstacle": "#314954",
        "obstacle_edge": "#58707a",
        "white": "#fffef9",
        "muted": "#91a3aa",
    }
    AGENT_COLORS = (
        "#27baa7",
        "#f26f48",
        "#e6c45b",
        "#57a6ff",
        "#c77dff",
        "#ff79a8",
        "#8bd450",
        "#22d3ee",
        "#ff9f1c",
        "#7f8cff",
        "#e76f51",
        "#4cc9f0",
        "#b8de6f",
        "#f28482",
    )

    OPEN_CELLS = set()
    OBSTACLES = set()
    AGENTS = {}
    PATHS = {}
    RAW_PATHS = {}
    MAKESPAN = 0
    VALIDATION_STATS = {}
    CBS_STATS = {}

    @classmethod
    def initialize_instance(cls):
        cls.OPEN_CELLS = {
            (row, column)
            for row in range(cls.GRID_SIZE)
            for column in range(cls.GRID_SIZE)
        }
        cls.OBSTACLES = {
            (row, column)
            for block_row, block_column in ((2, 2), (2, 7), (7, 2), (7, 7))
            for row in range(block_row, block_row + 3)
            for column in range(block_column, block_column + 3)
        }
        cls.OPEN_CELLS -= cls.OBSTACLES

        endpoints = (
            ("A", (5, 0), (5, 11)),
            ("B", (6, 11), (6, 0)),
            ("C", (0, 5), (11, 5)),
            ("D", (11, 6), (0, 6)),
            ("E", (4, 0), (10, 11)),
            ("F", (7, 11), (1, 0)),
            ("G", (0, 4), (11, 10)),
            ("H", (11, 7), (0, 1)),
            ("I", (1, 0), (10, 10)),
            ("J", (10, 11), (1, 1)),
            ("K", (0, 10), (11, 1)),
            ("L", (11, 1), (0, 10)),
            ("M", (0, 0), (11, 0)),
            ("N", (11, 11), (0, 11)),
        )
        cls.AGENTS = {
            name: {
                "start": start,
                "goal": goal,
                "color": cls.AGENT_COLORS[agent_index],
            }
            for agent_index, (name, start, goal) in enumerate(endpoints)
        }

        start_cells = [spec["start"] for spec in cls.AGENTS.values()]
        goal_cells = [spec["goal"] for spec in cls.AGENTS.values()]
        if len(set(start_cells)) != len(start_cells):
            raise ValueError("The CBS instance has duplicate start cells.")
        if len(set(goal_cells)) != len(goal_cells):
            raise ValueError("The CBS instance has duplicate goal cells.")
        if any(cell not in cls.OPEN_CELLS for cell in start_cells + goal_cells):
            raise ValueError("A start or goal lies on an obstacle.")

    @classmethod
    def manhattan_distance(cls, first_cell, second_cell):
        return sum(abs(first - second) for first, second in zip(first_cell, second_cell))

    @classmethod
    def reconstruct_path(cls, parent, final_state):
        path = []
        state = final_state
        while state is not None:
            path.append(state[0])
            state = parent[state]
        return list(reversed(path))

    @classmethod
    def goal_is_safe(cls, goal, arrival_time, vertex_constraints, edge_constraints):
        if any(time >= arrival_time and cell == goal for time, cell in vertex_constraints):
            return False
        return not any(
            time > arrival_time and source == goal and destination == goal
            for time, source, destination in edge_constraints
        )

    @classmethod
    def low_level_a_star(cls, agent_name, constraints):
        cls.CBS_STATS["low_level_calls"] += 1
        start = cls.AGENTS[agent_name]["start"]
        goal = cls.AGENTS[agent_name]["goal"]
        agent_constraints = [constraint for constraint in constraints if constraint[1] == agent_name]
        vertex_constraints = {
            (constraint[2], constraint[3])
            for constraint in agent_constraints
            if constraint[0] == "vertex"
        }
        edge_constraints = {
            (constraint[2], constraint[3], constraint[4])
            for constraint in agent_constraints
            if constraint[0] == "edge"
        }
        if (0, start) in vertex_constraints:
            return None

        frontier = []
        tie_breaker = count()
        start_state = (start, 0)
        heappush(
            frontier,
            (cls.manhattan_distance(start, goal), 0, next(tie_breaker), start_state),
        )
        parent = {start_state: None}
        discovered = {start_state}

        while frontier:
            _, _, _, (cell, time_step) = heappop(frontier)
            cls.CBS_STATS["low_level_expanded"] += 1

            if cell == goal and cls.goal_is_safe(
                goal,
                time_step,
                vertex_constraints,
                edge_constraints,
            ):
                return cls.reconstruct_path(parent, (cell, time_step))
            if time_step >= cls.MAX_LOW_LEVEL_TIME:
                continue

            row, column = cell
            candidates = [
                (row - 1, column),
                (row, column - 1),
                (row, column + 1),
                (row + 1, column),
                cell,
            ]
            candidates = [candidate for candidate in candidates if candidate in cls.OPEN_CELLS]

            next_time = time_step + 1
            for next_cell in candidates:
                if (next_time, next_cell) in vertex_constraints:
                    continue
                if (next_time, cell, next_cell) in edge_constraints:
                    continue

                next_state = (next_cell, next_time)
                if next_state in discovered:
                    continue
                discovered.add(next_state)
                parent[next_state] = (cell, time_step)
                heuristic = cls.manhattan_distance(next_cell, goal)
                heappush(
                    frontier,
                    (next_time + heuristic, next_time, next(tie_breaker), next_state),
                )
        return None

    @classmethod
    def path_position(cls, path, time_step):
        return path[min(time_step, len(path) - 1)]

    @classmethod
    def find_first_conflict(cls, paths):
        agent_names = sorted(paths)
        horizon = max(len(path) for path in paths.values())
        for time_step in range(horizon):
            occupied = {}
            for agent_name in agent_names:
                cell = cls.path_position(paths[agent_name], time_step)
                if cell in occupied:
                    return {
                        "type": "vertex",
                        "time": time_step,
                        "cell": cell,
                        "agents": (occupied[cell], agent_name),
                    }
                occupied[cell] = agent_name

            if time_step == 0:
                continue
            for first_agent, second_agent in combinations(agent_names, 2):
                first_previous = cls.path_position(paths[first_agent], time_step - 1)
                first_current = cls.path_position(paths[first_agent], time_step)
                second_previous = cls.path_position(paths[second_agent], time_step - 1)
                second_current = cls.path_position(paths[second_agent], time_step)
                if first_previous == second_current and second_previous == first_current:
                    return {
                        "type": "edge",
                        "time": time_step,
                        "agents": (first_agent, second_agent),
                        "moves": {
                            first_agent: (first_previous, first_current),
                            second_agent: (second_previous, second_current),
                        },
                    }
        return None

    @classmethod
    def count_conflicts(cls, paths):
        agent_names = sorted(paths)
        horizon = max(len(path) for path in paths.values())
        conflict_count = 0
        for time_step in range(horizon):
            occupied = defaultdict(int)
            for agent_name in agent_names:
                occupied[cls.path_position(paths[agent_name], time_step)] += 1
            conflict_count += sum(count_at_cell - 1 for count_at_cell in occupied.values() if count_at_cell > 1)

            if time_step == 0:
                continue
            for first_agent, second_agent in combinations(agent_names, 2):
                if (
                    cls.path_position(paths[first_agent], time_step - 1)
                    == cls.path_position(paths[second_agent], time_step)
                    and cls.path_position(paths[second_agent], time_step - 1)
                    == cls.path_position(paths[first_agent], time_step)
                ):
                    conflict_count += 1
        return conflict_count

    @classmethod
    def solution_cost(cls, paths):
        return sum(len(path) - 1 for path in paths.values())

    @classmethod
    def conflict_constraint(cls, conflict, agent_name):
        if conflict["type"] == "vertex":
            return ("vertex", agent_name, conflict["time"], conflict["cell"])
        source, destination = conflict["moves"][agent_name]
        return ("edge", agent_name, conflict["time"], source, destination)

    @classmethod
    def solve_with_cbs(cls):
        cls.CBS_STATS = {
            "high_level_expanded": 0,
            "high_level_generated": 1,
            "low_level_calls": 0,
            "low_level_expanded": 0,
        }
        root_constraints = tuple()
        root_paths = {}
        for agent_name in cls.AGENTS:
            path = cls.low_level_a_star(agent_name, root_constraints)
            if path is None:
                raise ValueError(f"No individual path exists for {agent_name}.")
            root_paths[agent_name] = path

        frontier = []
        tie_breaker = count()
        root_node = {
            "constraints": root_constraints,
            "paths": root_paths,
        }
        heappush(
            frontier,
            (
                cls.solution_cost(root_paths),
                next(tie_breaker),
                root_node,
            ),
        )

        while frontier:
            if cls.CBS_STATS["high_level_expanded"] >= cls.MAX_CBS_NODES:
                raise RuntimeError("CBS exceeded its high-level node limit.")
            _, _, node = heappop(frontier)
            cls.CBS_STATS["high_level_expanded"] += 1
            conflict = cls.find_first_conflict(node["paths"])
            if conflict is None:
                return node["paths"]

            for agent_name in conflict["agents"]:
                new_constraint = cls.conflict_constraint(conflict, agent_name)
                if new_constraint in node["constraints"]:
                    continue
                new_constraints = node["constraints"] + (new_constraint,)

                replanned_path = cls.low_level_a_star(agent_name, new_constraints)
                if replanned_path is None:
                    continue
                new_paths = dict(node["paths"])
                new_paths[agent_name] = replanned_path
                child_node = {
                    "constraints": new_constraints,
                    "paths": new_paths,
                }
                heappush(
                    frontier,
                    (
                        cls.solution_cost(new_paths),
                        next(tie_breaker),
                        child_node,
                    ),
                )
                cls.CBS_STATS["high_level_generated"] += 1
        raise RuntimeError("CBS exhausted its high-level search without a solution.")

    @classmethod
    def validate_timeline(cls, timeline, close_loop=False):
        if not timeline:
            raise ValueError("The MAPF timeline is empty.")

        agent_names = set(cls.AGENTS)
        for time_step, state in enumerate(timeline):
            if set(state) != agent_names:
                raise ValueError(f"Agent set changes at time {time_step}.")

            occupied = {}
            for agent_name, cell in state.items():
                if not all(isinstance(coordinate, int) for coordinate in cell):
                    raise ValueError(f"{agent_name} is not on a discrete grid cell at time {time_step}.")
                if cell not in cls.OPEN_CELLS:
                    raise ValueError(f"{agent_name} occupies an obstacle at time {time_step}.")
                if cell in occupied:
                    raise ValueError(
                        f"Vertex collision at time {time_step}: {occupied[cell]} and {agent_name}."
                    )
                occupied[cell] = agent_name

        transitions = list(zip(timeline, timeline[1:]))
        if close_loop:
            transitions.append((timeline[-1], timeline[0]))
        for transition_index, (current_state, next_state) in enumerate(transitions):
            for agent_name in agent_names:
                if cls.manhattan_distance(current_state[agent_name], next_state[agent_name]) > 1:
                    raise ValueError(
                        f"{agent_name} makes a nonlocal move at transition {transition_index}."
                    )
            for first_agent, second_agent in combinations(agent_names, 2):
                if (
                    current_state[first_agent] == next_state[second_agent]
                    and current_state[second_agent] == next_state[first_agent]
                ):
                    raise ValueError(
                        f"Edge-swap collision between {first_agent} and {second_agent} "
                        f"at transition {transition_index}."
                    )

    @classmethod
    def analyze_interactions(cls):
        visitors = defaultdict(set)
        waits = 0
        for agent_name, path in cls.RAW_PATHS.items():
            for cell in path:
                visitors[cell].add(agent_name)
            waits += sum(first == second for first, second in zip(path, path[1:]))

        shared_cells = {cell for cell, agents in visitors.items() if len(agents) > 1}
        handoffs = 0
        for time_step in range(cls.MAKESPAN):
            current_occupants = {
                cls.PATHS[agent_name][time_step]: agent_name
                for agent_name in cls.AGENTS
            }
            next_occupants = {
                cls.PATHS[agent_name][time_step + 1]: agent_name
                for agent_name in cls.AGENTS
            }
            handoffs += sum(
                cell in next_occupants and next_occupants[cell] != agent_name
                for cell, agent_name in current_occupants.items()
            )

        if not shared_cells:
            raise ValueError("The solved instance has no spatially shared cells.")
        if waits == 0:
            raise ValueError("The solved instance has no coordination waits.")
        if handoffs == 0:
            raise ValueError("The solved instance has no temporal cell handoffs.")
        return {
            "shared_cells": len(shared_cells),
            "coordination_waits": waits,
            "temporal_handoffs": handoffs,
        }

    @classmethod
    def validate_solution(cls, raw_paths):
        for agent_name, path in raw_paths.items():
            if path[0] != cls.AGENTS[agent_name]["start"]:
                raise ValueError(f"{agent_name} does not start at its assigned start cell.")
            if path[-1] != cls.AGENTS[agent_name]["goal"]:
                raise ValueError(f"{agent_name} does not finish at its assigned goal cell.")

        cls.RAW_PATHS = raw_paths
        cls.MAKESPAN = max(len(path) for path in raw_paths.values()) - 1
        cls.PATHS = {
            agent_name: path + [path[-1]] * (cls.MAKESPAN + 1 - len(path))
            for agent_name, path in raw_paths.items()
        }
        forward_timeline = [
            {agent_name: path[time_step] for agent_name, path in cls.PATHS.items()}
            for time_step in range(cls.MAKESPAN + 1)
        ]
        cls.validate_timeline(forward_timeline)

        animation_indices = list(range(cls.MAKESPAN + 1)) + list(range(cls.MAKESPAN - 1, 0, -1))
        animation_timeline = [forward_timeline[index] for index in animation_indices]
        cls.validate_timeline(animation_timeline, close_loop=True)
        cls.VALIDATION_STATS = cls.analyze_interactions()
        return animation_indices

    @classmethod
    def validate_continuous_motion(cls):
        minimum_separation = float("inf")
        for time_step in range(cls.MAKESPAN):
            for first_agent, second_agent in combinations(sorted(cls.AGENTS), 2):
                first_start = cls.PATHS[first_agent][time_step]
                first_end = cls.PATHS[first_agent][time_step + 1]
                second_start = cls.PATHS[second_agent][time_step]
                second_end = cls.PATHS[second_agent][time_step + 1]

                relative_start = tuple(
                    first_coordinate - second_coordinate
                    for first_coordinate, second_coordinate in zip(first_start, second_start)
                )
                relative_velocity = tuple(
                    (first_next - first_current) - (second_next - second_current)
                    for first_current, first_next, second_current, second_next in zip(
                        first_start,
                        first_end,
                        second_start,
                        second_end,
                    )
                )
                velocity_squared = sum(component * component for component in relative_velocity)
                if velocity_squared == 0:
                    minimizing_alpha = 0.0
                else:
                    minimizing_alpha = -sum(
                        position * velocity
                        for position, velocity in zip(relative_start, relative_velocity)
                    ) / velocity_squared
                    minimizing_alpha = max(0.0, min(1.0, minimizing_alpha))

                separation_squared = sum(
                    (position + minimizing_alpha * velocity) ** 2
                    for position, velocity in zip(relative_start, relative_velocity)
                )
                minimum_separation = min(minimum_separation, sqrt(separation_squared))

        required_separation = 2 * cls.AGENT_OUTER_RADIUS / cls.CELL_SIZE
        if minimum_separation <= required_separation:
            raise ValueError(
                "Interpolated robot markers overlap: "
                f"minimum separation {minimum_separation:.3f} cells, "
                f"required more than {required_separation:.3f}."
            )
        cls.VALIDATION_STATS["minimum_continuous_separation"] = minimum_separation

    @classmethod
    def animation_times(cls):
        keyframes = list(range(cls.MAKESPAN + 1)) + list(range(cls.MAKESPAN - 1, -1, -1))
        return [
            start_time + (end_time - start_time) * subframe / cls.FRAMES_PER_MOVE
            for start_time, end_time in zip(keyframes, keyframes[1:])
            for subframe in range(cls.FRAMES_PER_MOVE)
        ]

    @classmethod
    def interpolated_cell(cls, path, time_position):
        current_time = min(int(time_position), cls.MAKESPAN)
        next_time = min(current_time + 1, cls.MAKESPAN)
        interpolation = time_position - current_time
        return tuple(
            current + (following - current) * interpolation
            for current, following in zip(path[current_time], path[next_time])
        )

    @classmethod
    def cell_center(cls, cell):
        row, column = cell
        origin_x, origin_y = cls.GRID_ORIGIN
        return (
            origin_x + column * cls.CELL_SIZE + cls.CELL_SIZE // 2,
            origin_y + row * cls.CELL_SIZE + cls.CELL_SIZE // 2,
        )

    @classmethod
    def font(cls, size, bold=False):
        font_name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
        try:
            return ImageFont.truetype(font_name, size)
        except OSError:
            return ImageFont.load_default(size=size)

    @classmethod
    def draw_goal(cls, draw, center, color):
        radius = 11
        bounds = (
            center[0] - radius,
            center[1] - radius,
            center[0] + radius,
            center[1] + radius,
        )
        for start_angle in range(0, 360, 60):
            draw.arc(bounds, start=start_angle, end=start_angle + 35, fill=color, width=3)

    @classmethod
    def draw_frame(cls, time_position):
        image = Image.new("RGB", (cls.CANVAS_SIZE, cls.CANVAS_SIZE), cls.COLORS["background"])
        draw = ImageDraw.Draw(image)
        origin_x, origin_y = cls.GRID_ORIGIN
        grid_side = cls.GRID_SIZE * cls.CELL_SIZE

        time_label = f"t = {time_position:05.2f} / {cls.MAKESPAN}"
        time_font = cls.font(18, bold=True)
        time_bounds = draw.textbbox((0, 0), time_label, font=time_font)
        draw.text(
            (origin_x + grid_side - (time_bounds[2] - time_bounds[0]), 38),
            time_label,
            fill=cls.COLORS["white"],
            font=time_font,
        )

        draw.rounded_rectangle(
            (origin_x, origin_y, origin_x + grid_side, origin_y + grid_side),
            radius=22,
            fill=cls.COLORS["panel"],
            outline=cls.COLORS["grid"],
            width=2,
        )

        for row, column in cls.OBSTACLES:
            left = origin_x + column * cls.CELL_SIZE + 2
            top = origin_y + row * cls.CELL_SIZE + 2
            right = left + cls.CELL_SIZE - 4
            bottom = top + cls.CELL_SIZE - 4
            draw.rounded_rectangle(
                (left, top, right, bottom),
                radius=5,
                fill=cls.COLORS["obstacle"],
                outline=cls.COLORS["obstacle_edge"],
                width=1,
            )

        for index in range(1, cls.GRID_SIZE):
            offset = index * cls.CELL_SIZE
            draw.line(
                (origin_x + offset, origin_y, origin_x + offset, origin_y + grid_side),
                fill=cls.COLORS["grid"],
                width=1,
            )
            draw.line(
                (origin_x, origin_y + offset, origin_x + grid_side, origin_y + offset),
                fill=cls.COLORS["grid"],
                width=1,
            )

        for agent_name, spec in cls.AGENTS.items():
            cls.draw_goal(draw, cls.cell_center(spec["goal"]), spec["color"])

        for agent_name, path in cls.PATHS.items():
            center = cls.cell_center(cls.interpolated_cell(path, time_position))
            current_time = min(int(time_position), cls.MAKESPAN)
            next_time = min(current_time + 1, cls.MAKESPAN)
            if current_time < cls.MAKESPAN and path[current_time] == path[next_time]:
                wait_radius = 17
                draw.ellipse(
                    (
                        center[0] - wait_radius,
                        center[1] - wait_radius,
                        center[0] + wait_radius,
                        center[1] + wait_radius,
                    ),
                    outline=cls.COLORS["muted"],
                    width=2,
                )

            outer_radius = cls.AGENT_OUTER_RADIUS
            inner_radius = 10
            draw.ellipse(
                (
                    center[0] - outer_radius,
                    center[1] - outer_radius,
                    center[0] + outer_radius,
                    center[1] + outer_radius,
                ),
                fill=cls.COLORS["white"],
            )
            draw.ellipse(
                (
                    center[0] - inner_radius,
                    center[1] - inner_radius,
                    center[0] + inner_radius,
                    center[1] + inner_radius,
                ),
                fill=cls.AGENTS[agent_name]["color"],
            )
        return image

    @classmethod
    def write_gif(cls, frame_times):
        frames = [cls.draw_frame(time_position) for time_position in frame_times]
        durations = [70] * len(frames)
        durations[0] = 700
        goal_frame = cls.MAKESPAN * cls.FRAMES_PER_MOVE
        durations[goal_frame] = 900

        cls.OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
        frames[0].save(
            cls.OUTPUT_PATH,
            save_all=True,
            append_images=frames[1:],
            duration=durations,
            loop=0,
            disposal=1,
            optimize=True,
        )

        with Image.open(cls.OUTPUT_PATH) as generated_gif:
            if generated_gif.n_frames != len(frames):
                raise ValueError("Generated GIF has an unexpected frame count.")
            for frame_index in range(generated_gif.n_frames):
                generated_gif.seek(frame_index)
                generated_gif.load()
                if generated_gif.size != (cls.CANVAS_SIZE, cls.CANVAS_SIZE):
                    raise ValueError(f"GIF frame {frame_index} has unexpected dimensions.")

    @classmethod
    def generate(cls):
        cls.initialize_instance()
        raw_paths = cls.solve_with_cbs()
        cls.validate_solution(raw_paths)
        cls.validate_continuous_motion()
        frame_times = cls.animation_times()
        cls.write_gif(frame_times)

        print(
            f"CBS solved {len(cls.AGENTS)} agents on a {cls.GRID_SIZE}x{cls.GRID_SIZE} grid: "
            f"sum of costs {cls.solution_cost(raw_paths)}, makespan {cls.MAKESPAN}, "
            f"{cls.CBS_STATS['high_level_expanded']} high-level nodes expanded."
        )
        print(
            f"Validated every timestep: {cls.VALIDATION_STATS['shared_cells']} shared cells, "
            f"{cls.VALIDATION_STATS['coordination_waits']} coordination waits, "
            f"{cls.VALIDATION_STATS['temporal_handoffs']} temporal handoffs; "
            f"minimum interpolated separation "
            f"{cls.VALIDATION_STATS['minimum_continuous_separation']:.3f} cells; "
            "no obstacle, vertex, edge-swap, or continuous marker conflicts."
        )
        print(
            f"Wrote {cls.OUTPUT_PATH} "
            f"({len(frame_times)} frames, {cls.CANVAS_SIZE}x{cls.CANVAS_SIZE})."
        )


if __name__ == "__main__":
    MapfSolutionGif.generate()
    