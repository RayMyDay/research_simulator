import heapq
import math
from constants import LEFT_BOUNDARY, RIGHT_BOUNDARY, TOP_BOUNDARY, BOTTOM_BOUNDARY

class Node:
    def __init__(self, position, g=0, h=0):
        self.position = position
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f

class ControllerAStar:
    def __init__(self, agent, walls):
        self.agent = agent
        self.walls = walls
        self.goal = None
        self.start = None
        self.path = []
        self.current_target_index = 0
        self.running = False

    def heuristic(self, a, b):
        dx = abs(b[0] - a[0])
        dy = abs(b[1] - a[1])
        return dx + dy + (math.sqrt(2) - 2) * min(dx, dy)

    def get_neighbors(self, current):
        x, y = current
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (-1, -1), (1, -1), (-1, 1)
        ]
        neighbors = []
        for dx, dy in directions:
            neighbor = (x + dx, y + dy)
            if self.is_valid_position(neighbor):
                neighbors.append(neighbor)
        return neighbors

    def is_valid_position(self, position):
        x, y = position
        BUFFER_DISTANCE = 10  # Define the buffer distance from walls

        # Check boundaries
        if not (LEFT_BOUNDARY + BUFFER_DISTANCE <= x <= RIGHT_BOUNDARY - BUFFER_DISTANCE and
                TOP_BOUNDARY + BUFFER_DISTANCE <= y <= BOTTOM_BOUNDARY - BUFFER_DISTANCE):
            return False

        # Check collision with walls, considering the buffer distance
        for wall in self.walls:
            if wall.is_colliding(x, y, self.agent.body_radius + BUFFER_DISTANCE):
                return False

        # Return True if all checks are passed
        return True



    def find_path(self):
        start = (int(self.agent.x), int(self.agent.y))
        goal = (int(self.goal[0]), int(self.goal[1]))

        if not self.is_valid_position(start) or not self.is_valid_position(goal):
            print(f"Start {start} or goal {goal} is invalid.")
            return []

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            # If the goal is reached
            if self.is_goal_reached(current, goal):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                print(f"Path found with {len(path)} points.")
                self.path = path
                return path

            # Explore neighbors
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        print("No path found.")
        self.path = []
        return []


    def move_towards(self, target_x, target_y):
        direction_x = target_x - self.agent.x
        direction_y = target_y - self.agent.y
        distance = math.sqrt(direction_x ** 2 + direction_y ** 2)

        if distance == 0:
            print("Agent already at the target position.")
            return

        direction_x /= distance
        direction_y /= distance

        speed = min(self.agent.linear_speed, distance)
        next_x = self.agent.x + direction_x * speed
        next_y = self.agent.y + direction_y * speed

        if not self.agent.will_collide(next_x, next_y):
            self.agent.x = next_x
            self.agent.y = next_y
        else:
            print(f"Movement blocked at ({next_x}, {next_y}).")

    def is_goal_reached(self, current_pos, goal_pos, tolerance=1):
        return (abs(current_pos[0] - goal_pos[0]) <= tolerance and
                abs(current_pos[1] - goal_pos[1]) <= tolerance)
        
    def set_goal(self, goal):
        self.goal = goal
        self.find_path()
        self.current_target_index = 0

        if not self.path:
            print("No valid path found. Cannot start navigation.")
            self.running = False
        else:
            self.running = True

    def update(self):
        if not self.running or not self.path:
            print("Agent is not running or no valid path to follow.")
            return

        if self.current_target_index >= len(self.path):
            print("Agent reached the goal.")
            self.running = False
            return

        target = self.path[self.current_target_index]
        target_x, target_y = target

        self.move_towards(target_x, target_y)

        dx = target_x - self.agent.x
        dy = target_y - self.agent.y
        distance = math.sqrt(dx**2 + dy**2)
        REACH_THRESHOLD = 10

        if distance <= REACH_THRESHOLD:
            self.current_target_index += 1

            if self.current_target_index >= len(self.path):
                print("Path completed.")
                self.running = False

    def simplify_path(self, path):
        if len(path) < 3:
            return path  # No simplification needed for short paths

        simplified_path = [path[0]]  # Always include the start point

        for i in range(1, len(path) - 1):
            prev = simplified_path[-1]
            curr = path[i]
            next_point = path[i + 1]

            # Check if the line from prev to next_point is valid
            if not self.has_line_of_sight(prev, next_point):
                simplified_path.append(curr)  # Add the current point if no line-of-sight

        simplified_path.append(path[-1])  # Always include the goal

        # Ensure we always return at least the start and goal
        if len(simplified_path) < 2:
            return path
        return simplified_path


    def has_line_of_sight(self, start, end):
        x0, y0 = start
        x1, y1 = end

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        while (x0, y0) != (x1, y1):
            if not self.is_valid_position((x0, y0)):
                return False

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return self.is_valid_position(end)


    def handle_input(self, mouse_pos):
        snapped_pos = (round(mouse_pos[0]), round(mouse_pos[1]))

        if self.is_valid_position(snapped_pos):
            print(f"Goal set to: {snapped_pos}")
            self.set_goal(snapped_pos)
        else:
            print(f"Invalid goal position: {mouse_pos}")
            self.running = False