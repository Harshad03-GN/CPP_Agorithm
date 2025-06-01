import pygame
import random
import heapq

class RobotNavigation:
    def __init__(self, width, height, obstacles):
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

        self.robot_pos = [0, 0]
        self.visited_cells = set([(0, 0)])
        self.unvisited_cells = set((x, y) for x in range(width)
                                    for y in range(height)
                                    if (x, y) != (0, 0))

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
                    self.dynamic_obstacles.append(obstacle)
                    break

        # initial position
        self.grid[0][0] = self.ROBOT

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
                    self.dynamic_obstacles[i] = [new_x, new_y]
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

    def find_most_efficient_path(self):
        """
        Find the most efficient path to an unvisited cell
        """
        start = tuple(self.robot_pos)

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

    def move_robot(self, path):
        """
        Move robot along the calculated path
        """
        if not path:
            return False

        for x, y in path[:-1]:
            if self.grid[y][x] == self.VISITED:
                self.grid[y][x] = self.RETRACED_PATH

        x, y = path[-1]
        self.grid[self.robot_pos[1]][self.robot_pos[0]] = self.VISITED
        self.grid[y][x] = self.ROBOT

        self.robot_pos = [x, y]

        if (x, y) in self.unvisited_cells:
            self.unvisited_cells.remove((x, y))
        self.visited_cells.add((x, y))

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

    def draw_path(self, path, robot_nav):
        if len(path) > 1:
            for i in range(len(path) - 1):
                start_pos = (path[i][0] * self.CELL_SIZE + self.CELL_SIZE // 2, path[i][1] * self.CELL_SIZE + self.CELL_SIZE // 2)
                end_pos = (path[i + 1][0] * self.CELL_SIZE + self.CELL_SIZE // 2, path[i + 1][1] * self.CELL_SIZE + self.CELL_SIZE // 2)
                pygame.draw.line(
                    self.screen,
                    (255, 0, 0),
                    start_pos,
                    end_pos,
                    3
                )

def run_simulation(width, height, obstacles):
    robot_nav = RobotNavigation(width, height, obstacles)
    visualization = GridVisualization(width, height)

    clock = pygame.time.Clock()
    running = True
    path_traveled = []

    robot_delay = 800
    dynamic_obstacle_delay = 800
    waiting_time = 1000  # Time to wait when encountering a dynamic obstacle (in ms)

    last_robot_move = pygame.time.get_ticks()
    last_obstacle_move = pygame.time.get_ticks()
    waiting_start_time = None  # Track when the robot starts waiting

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        current_time = pygame.time.get_ticks()

        if current_time - last_obstacle_move > dynamic_obstacle_delay:
            robot_nav.move_dynamic_obstacles()
            last_obstacle_move = current_time

        # Check if the robot is waiting
        if waiting_start_time:
            if current_time - waiting_start_time >= waiting_time:
                waiting_start_time = None  # Robot stops waiting after enough time
            else:
                # Skip robot movement while waiting
                visualization.screen.fill((255, 255, 255))
                visualization.draw_grid(robot_nav)
                visualization.draw_path(path_traveled, robot_nav)
                pygame.display.flip()
                clock.tick(60)
                continue

        if current_time - last_robot_move > robot_delay:
            path = robot_nav.find_most_efficient_path()

            if path:
                # Check if a dynamic obstacle is blocking the path
                blocked = False
                for x, y in path:
                    if robot_nav.grid[y][x] == robot_nav.DYNAMIC_OBSTACLE:
                        blocked = True
                        break

                if blocked:
                    # Start waiting if the path is blocked
                    if not waiting_start_time:
                        waiting_start_time = current_time
                else:
                    # Move the robot if the path is clear
                    robot_nav.move_robot(path)
                    path_traveled.extend(path)

            if robot_nav.is_exploration_complete():
                # Mark the robot's final position as visited and let it rest there
                robot_nav.grid[robot_nav.robot_pos[1]][robot_nav.robot_pos[0]] = robot_nav.ROBOT
                print("Grid fully explored!")
                visualization.screen.fill((255, 255, 255))
                visualization.draw_grid(robot_nav)
                visualization.draw_path(path_traveled, robot_nav)
                pygame.display.flip()
                pygame.time.wait(2000)  # Pause to display the final position
                break


            last_robot_move = current_time

        visualization.screen.fill((255, 255, 255))
        visualization.draw_grid(robot_nav)
        visualization.draw_path(path_traveled, robot_nav)
        pygame.display.flip()

        clock.tick(60)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        pygame.display.flip()

    pygame.quit()

def main():
    width = 10
    height = 10
    obstacles = [(2, 2), (3, 4), (7, 7)]
    run_simulation(width, height, obstacles)

if __name__ == "__main__":
    main()
