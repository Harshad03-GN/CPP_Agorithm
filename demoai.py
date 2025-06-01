import pygame
import random
import heapq

def make_hashable(position):
    """
    Convert lists to tuples for hashable usage.
    """
    return tuple(position)

class RobotNavigation:
    def __init__(self, width, height, obstacles, robot_positions):
        """
        Initialize grid environment with efficient navigation
        """
        self.width = width
        self.height = height
        self.grid = [[0 for _ in range(width)] for _ in range(height)]

        self.UNVISITED = 0
        self.VISITED = 1
        self.OBSTACLE = 2
        self.ROBOT = 3
        self.RETRACED_PATH = 4
        self.DYNAMIC_OBSTACLE = 5

        self.robot_positions = [make_hashable(pos) for pos in robot_positions]
        self.visited_cells = set(self.robot_positions)
        self.unvisited_cells = set((x, y) for x in range(width)
                                    for y in range(height)
                                    if (x, y) not in self.robot_positions)

        # for static obstacles
        for x, y in obstacles:
            self.grid[y][x] = self.OBSTACLE
            if (x, y) in self.unvisited_cells:
                self.unvisited_cells.remove((x, y))

        # for dynamic obstacles
        self.dynamic_obstacles = []
        for _ in range(5):
            while True:
                obstacle = [random.randint(0, width - 1), random.randint(0, height - 1)]
                if self.grid[obstacle[1]][obstacle[0]] == self.UNVISITED:
                    self.grid[obstacle[1]][obstacle[0]] = self.DYNAMIC_OBSTACLE
                    self.dynamic_obstacles.append(make_hashable(obstacle))
                    break

        # initial robot positions
        for x, y in self.robot_positions:
            self.grid[y][x] = self.ROBOT

        self.robot_paths = {i: [] for i in range(len(robot_positions))}  # track paths for each robot

    def move_dynamic_obstacles(self):
        """
        Move the dynamic obstacles randomly but not blocking essential paths
        """
        for i, obstacle in enumerate(self.dynamic_obstacles):
            x, y = obstacle

            # cell state updation
            if (x, y) in self.visited_cells:
                self.grid[y][x] = self.VISITED
            elif (x, y) not in self.unvisited_cells:
                self.grid[y][x] = self.RETRACED_PATH
            else:
                self.grid[y][x] = self.UNVISITED

            # direction locator
            directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
            random.shuffle(directions)
            for dx, dy in directions:
                new_x, new_y = x + dx, y + dy
                if (0 <= new_x < self.width and
                        0 <= new_y < self.height and
                        self.grid[new_y][new_x] in [self.UNVISITED, self.VISITED, self.RETRACED_PATH] and
                        self.grid[new_y][new_x] != self.ROBOT):  # collision avoidance
                    self.dynamic_obstacles[i] = (new_x, new_y)
                    self.grid[new_y][new_x] = self.DYNAMIC_OBSTACLE
                    break

    def astar_pathfinding(self, start, target):
        """
        A* pathfinding algorithm to find the shortest path from start to target
        """
        open_set = []
        heapq.heappush(open_set, (0, start, []))
        g_score = {start: 0}
        visited = set()

        while open_set:
            _, current, path = heapq.heappop(open_set)

            if current == target:
                return path + [current]

            if current in visited:
                continue

            visited.add(current)

            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                neighbor = (current[0] + dx, current[1] + dy)

                if (0 <= neighbor[0] < self.width and
                        0 <= neighbor[1] < self.height and
                        self.grid[neighbor[1]][neighbor[0]] not in [self.OBSTACLE, self.DYNAMIC_OBSTACLE]):

                    tentative_g_score = g_score[current] + 1

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + abs(neighbor[0] - target[0]) + abs(neighbor[1] - target[1])
                        heapq.heappush(open_set, (f_score, neighbor, path + [current]))

        return None

    def find_most_efficient_path(self, robot_index):
        """
        Find the most efficient path to an unvisited cell for the specified robot
        """
        start = self.robot_positions[robot_index]

        # If there's only one unvisited cell, find the direct path
        if len(self.unvisited_cells) == 1:
            target = next(iter(self.unvisited_cells))
            return self.astar_pathfinding(start, target)

        # Normal A* algorithm to find the most efficient path
        open_set = []
        heapq.heappush(open_set, (0, start, []))
        visited = set([start])

        while open_set:
            priority, current, path = heapq.heappop(open_set)

            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                neighbor = (current[0] + dx, current[1] + dy)

                if (0 <= neighbor[0] < self.width and
                        0 <= neighbor[1] < self.height and
                        self.grid[neighbor[1]][neighbor[0]] not in [self.OBSTACLE, self.DYNAMIC_OBSTACLE] and
                        neighbor not in visited):

                    new_path = path + [neighbor]

                    if neighbor in self.unvisited_cells:
                        return new_path

                    visited.add(neighbor)

                    unexplored_neighbors = sum(
                        1 for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]
                        if (0 <= neighbor[0] + dx < self.width and
                            0 <= neighbor[1] + dy < self.height and
                            self.grid[neighbor[1] + dy][neighbor[0] + dx] == self.UNVISITED)
                    )
                    priority = len(new_path) - unexplored_neighbors
                    heapq.heappush(open_set, (priority, neighbor, new_path))

        return None

    def move_robot(self, robot_index, path):
        """
        Move robot along the calculated path
        """
        if not path:
            return False

        for x, y in path[:-1]:
            if self.grid[y][x] == self.VISITED:
                self.grid[y][x] = self.RETRACED_PATH

        x, y = path[-1]
        self.grid[self.robot_positions[robot_index][1]][self.robot_positions[robot_index][0]] = self.VISITED
        self.grid[y][x] = self.ROBOT

        self.robot_positions[robot_index] = (x, y)

        if (x, y) in self.unvisited_cells:
            self.unvisited_cells.remove((x, y))
        self.visited_cells.add((x, y))

        self.robot_paths[robot_index].append((x, y))  # track path

        return True

    def is_exploration_complete(self):
        """
        Check if entire grid is explored
        """
        return not self.unvisited_cells

class GridVisualization:
    def __init__(self, width, height):
        """
        Initialize Pygame visualization
        """
        pygame.init()

        self.CELL_SIZE = max(20, min(800 // max(width, height), 40))
        self.SCREEN_WIDTH = width * self.CELL_SIZE
        self.SCREEN_HEIGHT = height * self.CELL_SIZE

        self.COLORS = {
            0: (255, 255, 255),
            1: (200, 255, 200),
            2: (100, 100, 100),
            3: (0, 0, 255),
            4: (255, 255, 0),
            5: (255, 0, 0)
        }

        self.PATH_COLORS = [
            (0, 255, 0),  # Robot 0
            (0, 255, 255),  # Robot 1
            (255, 0, 255)  # Robot 2
        ]

        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        pygame.display.set_caption('Coverage Path Planning Grid')

    def draw_grid(self, robot_nav):
        for y in range(robot_nav.height):
            for x in range(robot_nav.width):
                rect = pygame.Rect(
                    x * self.CELL_SIZE,
                    y * self.CELL_SIZE,
                    self.CELL_SIZE,
                    self.CELL_SIZE
                )
                pygame.draw.rect(
                    self.screen,
                    self.COLORS[robot_nav.grid[y][x]],
                    rect
                )
                pygame.draw.rect(
                    self.screen,
                    (0, 0, 0),
                    rect,
                    1
                )

        # Draw robot paths
        for robot_index, path in robot_nav.robot_paths.items():
            for x, y in path:
                rect = pygame.Rect(
                    x * self.CELL_SIZE + self.CELL_SIZE // 4,
                    y * self.CELL_SIZE + self.CELL_SIZE // 4,
                    self.CELL_SIZE // 2,
                    self.CELL_SIZE // 2
                )
                pygame.draw.rect(self.screen, self.PATH_COLORS[robot_index], rect)

def run_simulation(width, height, obstacles):
    robot_positions = [[0, 0], [0, height - 1], [width - 1, 0]]
    robot_nav = RobotNavigation(width, height, obstacles, robot_positions)
    visualization = GridVisualization(width, height)

    clock = pygame.time.Clock()
    running = True
    robot_delay = 1000
    dynamic_obstacle_delay = 1000

    last_robot_move = pygame.time.get_ticks()
    last_obstacle_move = pygame.time.get_ticks()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        current_time = pygame.time.get_ticks()

        if current_time - last_obstacle_move > dynamic_obstacle_delay:
            robot_nav.move_dynamic_obstacles()
            last_obstacle_move = current_time

        if current_time - last_robot_move > robot_delay:
            for i in range(len(robot_positions)):
                path = robot_nav.find_most_efficient_path(i)
                if path:
                    robot_nav.move_robot(i, path)

            if robot_nav.is_exploration_complete():
                print("Grid fully explored!")
                break

            last_robot_move = current_time

        visualization.screen.fill((255, 255, 255))
        visualization.draw_grid(robot_nav)
        pygame.display.flip()

        clock.tick(60)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        pygame.display.flip()

    pygame.quit()

def main():
    width = 15
    height = 15
    obstacles = [(5, 5), (10, 10), (14, 14), (7, 7)]
    run_simulation(width, height, obstacles)

if __name__ == "__main__":
    main()

