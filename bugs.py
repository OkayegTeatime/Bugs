import math
import time
from itertools import count
import matplotlib.pyplot as plt


def find_line_equation(point1, point2):
    buffer = 5
    x1, y1 = point1
    x2, y2 = point2
    if x1 == x2:
        a, b, c = 1, 0, x1
    else:
        m = (y2 - y1) / (x2 - x1)
        a = -m
        b = 1
        c = a * x1 + b * y1
    x_limit = [x1, x2]
    x_limit.sort()
    y_limit = [y1, y2]
    y_limit.sort()
    edge = {'coefficients': [a, b, c],
            'limit': {'x': [x_limit[0] - buffer, x_limit[1] + buffer],
                      'y': [y_limit[0] - buffer, y_limit[1] + buffer]},
            'points': [point1, point2]}
    return edge


def farthest_point(coord1, coord2, target_coord):
    distance1 = math.sqrt((coord1[0] - target_coord[0]) ** 2 + (coord1[1] - target_coord[1]) ** 2)
    distance2 = math.sqrt((coord2[0] - target_coord[0]) ** 2 + (coord2[1] - target_coord[1]) ** 2)
    if distance1 > distance2:
        return coord1
    else:
        return coord2


class Bug:
    def __init__(self, start, goal, edges, bug_type, steps=500, step_distance=1):
        self.bug_type = bug_type
        self.x = start[0]
        self.y = start[1]
        self.start = start
        self.goal = goal
        self.heading = 0
        self.position_history = [[self.x, self.y]]
        self.mode = 'goal'
        self.lockout_max = 5
        self.lockout = 0
        self.m_line = find_line_equation(start, goal)
        self.entry_points = []
        self.entry_point_indexes = []
        self.m_line_intercepts = []
        self.step_distance = step_distance
        self.steps = steps
        self.edges = edges
        self.exit_point = None
        self.time_step = 0

    def run(self):
        print(f"Bug {self.bug_type} Starting Location:", self.get_position())
        start_time = time.time()
        self.turn_to_goal()
        for _ in range(0, self.steps):
            self.move_forward(self.step_distance)
            self.detect_edges(self.edges)
            if self.bug_type == 1: self.detect_entry_exit_point()
            if self.bug_type == 2: self.detect_m_line()
            if self._detect_point(self.goal, threshold=0.003):
                print('GOAL REACHED')
                runtime = round((time.time() - start_time), 2)
                print(f'\nBug {self.bug_type} sim completed with {len(self.position_history)} steps in {runtime} sec.')
                print('Entry Points: ', [[int(x) for x in point] for point in self.entry_points])
                print('M-Line Intercepts: ', [[int(x) for x in point] for point in self.m_line_intercepts])
                self.spiral(self.goal)
                break

    def move_forward(self, distance):
        self.x += distance * math.cos(self.heading)
        self.y += distance * math.sin(self.heading)
        self.position_history.append(self.get_position())
        self.lockout -= 1
        self.time_step += 1

    def turn(self, angle):
        self.heading += angle

    def get_position(self):
        return [self.x, self.y]

    def _rewind_position(self):
        previous_location = self.position_history[-2]
        self.position_history[-1] = self.x, self.y = previous_location

    def turn_to_goal(self):
        self.heading = math.atan((self.y - self.goal[1]) / (self.x - self.goal[0]))

    def detect_edges(self, edges):
        if self.lockout < 0:
            for edge in edges:
                if self._find_collision(edge):
                    if self.mode == 'goal':
                        self.entry_points.append(self.get_position())
                        self.entry_point_indexes.append(self.time_step)
                        self.mode = 'circ'
                    print('\nCrossed edge')
                    self.lockout = self.lockout_max
                    self.turn_along_edge(edge)

    def detect_m_line(self):
        if self.mode == 'circ':
            if self.lockout < 0:
                if self._find_collision(self.m_line):
                    self.lockout = self.lockout_max
                    print('\nCrossed M-line')
                    self.m_line_intercepts.append(self.get_position())
                    if farthest_point(self.entry_points[-1], self.get_position(), self.goal) == self.entry_points[-1]:
                        self._find_direction_to_turn(self.m_line, m_line=True)
                        self.mode = 'goal'

    def turn_along_edge(self, edge):
        if self.mode == 'circ' or self.mode == 'exiting':
            self._find_direction_to_turn(edge)
        else:
            self._rewind_position()
            self.heading = math.atan(-edge['coefficients'][0])

    def detect_entry_exit_point(self):
        if self.mode == 'circ':
            if self.lockout < -5:
                is_complete = self._detect_point(self.entry_points[-1])
                if is_complete:
                    print('Completed circumnavigation at point: ', self.entry_points[-1])
                    self.lockout = self.lockout_max
                    closest_point_goal = [10000000, 10000000]
                    points_on_poly = self.position_history[self.entry_point_indexes[-1]:]
                    for point in points_on_poly:
                        farther = farthest_point(closest_point_goal, point, self.goal)
                        if farther == closest_point_goal:
                            closest_point_goal = point
                    self.exit_point = closest_point_goal
                    self.mode = 'exiting'
                    if points_on_poly.index(closest_point_goal) < len(points_on_poly) / 2:
                        self.heading += math.pi
        elif self.mode == 'exiting':
            if self._detect_point(self.exit_point, threshold=0.005):
                print(f'Exiting at point: {self.exit_point}')
                self.lockout = self.lockout_max
                self.turn_to_goal()
                self.mode = 'goal'

    def _detect_point(self, point, threshold=0.01):
        x, y = self.get_position()
        target_x, target_y = point
        distance = math.sqrt((x - target_x) ** 2 + (y - target_y) ** 2)
        norm = math.sqrt((self.start[0] - self.goal[0]) ** 2 + (self.start[1] - self.goal[1]) ** 2)
        return distance / norm < threshold

    # def go_to_m_line(self):

    def _find_collision(self, edge):
        if edge['limit']['x'][0] < self.x < edge['limit']['x'][1] and edge['limit']['y'][0] < self.y < edge['limit']['y'][1]:
            x, y = self.position_history[-2]
            previous_side = edge['coefficients'][0] * x + edge['coefficients'][1] * y < edge['coefficients'][2]
            x, y = self.position_history[-1]
            current_side = edge['coefficients'][0] * x + edge['coefficients'][1] * y < edge['coefficients'][2]
            return previous_side != current_side

    def _find_direction_to_turn(self, edge, m_line=False):
        if m_line:
            farthest_vertex = self.goal
        else:
            farthest_vertex = farthest_point(edge['points'][0], edge['points'][1], [self.x, self.y])
        print(farthest_vertex)
        guess_heading = math.atan(-edge['coefficients'][0])
        projected = [self.x + math.cos(guess_heading), self.y + math.sin(guess_heading)]
        farther_point = farthest_point([self.x, self.y], projected, farthest_vertex)
        if projected == farther_point:
            self.heading = guess_heading + math.pi
        else:
            self.heading = guess_heading
        pass

    def spiral(self, origin=None):
        if not origin: origin = [0, 0]
        a = 0
        for i in range(1, 300):
            a += math.atan(1 / math.sqrt(i))
            r = math.sqrt(i + 1)
            x = r * math.cos(a) + origin[0]
            y = r * math.sin(a) + origin[1]
            self.position_history.append([x, y])


class Anim:
    def __init__(self, data, start, goal, edges, title='Bug Motion'):
        self.title = title
        self.x = []
        self.y = []
        self.data = data
        self.fig, self.ax = plt.subplots()
        self.counter = count(0, 10)
        self.start = start
        self.goal = goal
        self.edges = edges

    def update_plot(self, frame):
        ind = next(self.counter)
        if 1 < ind < len(self.data) - 1:
            self.x = [point[0] for point in self.data[0:ind]]
            self.y = [point[1] for point in self.data[0:ind]]
            plt.cla()
            self.ax.plot(self.x, self.y)
            self.ax.plot(self.x[-1], self.y[-1], marker='o')
            self.ax.plot([self.start[0], self.goal[0]], [self.start[1], self.goal[1]], marker='o')
            for edge in self.edges:
                self.ax.plot([point[0] for point in edge['points']], [point[1] for point in edge['points']], color='red')
            plt.title(self.title)
            plt.xlim([0, 780])
            plt.ylim([0, 400])


# Example usage:
if __name__ == "__main__":
    pass
    # start_point = [75, 224]
    # goal_point = [682, 223]
    # edges = []
    # contours = find_contours(image)
    # for contour in contours:
    #     vertices = find_vertices(contour)
    #     for ind in range(len(vertices) - 1):
    #         edges.append(find_line_equation(vertices[ind], vertices[ind + 1]))
    # bug1 = Bug(start_point, goal_point, 1, steps=2000, step_distance=2)
    # bug1.run()
    # bug2 = Bug(start_point, goal_point, 2, steps=1500, step_distance=1)
    # bug2.run()
    # animation = Anim(bug1.position_history[0::3], title=f'Bug Motion')
    # anim = FuncAnimation(animation.fig, animation.update_plot, blit=False, interval=10, save_count=1000)
    # plt.show()
