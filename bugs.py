import math
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from process_image import image, find_contours, find_vertices


def find_line_equation(point1, point2):
    # Calculate the slope (m) of the line
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
    # print(f'\nEquation of line segment: {a}x + {b}y = {c} \nfor x in {x_limit} and y in {y_limit}')
    return edge


def farthest_point(coord1, coord2, target_coord):
    distance1 = math.sqrt((coord1[0] - target_coord[0]) ** 2 + (coord1[1] - target_coord[1]) ** 2)
    distance2 = math.sqrt((coord2[0] - target_coord[0]) ** 2 + (coord2[1] - target_coord[1]) ** 2)
    if distance1 > distance2:
        return coord1
    else:
        return coord2


class Bug1:
    def __init__(self, start, goal):
        self.x = start[0]
        self.y = start[1]
        self.goal = goal
        self.heading = 0
        self.position_history = [(self.x, self.y)]
        self.current_mode = 'goal'
        self.lockout = 0
        self.m_line = find_line_equation(start_point, goal_point)

    def move_forward(self, distance):
        self.x += distance * math.cos(self.heading)
        self.y += distance * math.sin(self.heading)
        self.position_history.append((self.x, self.y))
        self.lockout -= 1

    def turn(self, angle):
        self.heading += angle

    def get_position(self):
        return self.x, self.y

    def _rewind_position(self):
        previous_location = self.position_history[-2]
        self.position_history[-1] = self.x, self.y = previous_location

    def turn_to_goal(self):
        self.heading = math.atan((self.y - self.goal[1]) / (self.x - self.goal[0]))

    def detect_edges(self, edges):
        if self.lockout < 0:
            for edge in edges:
                if self._find_collision(edge):
                    self.lockout = 2
                    print('Crossed edge\n', edge)
                    self.turn_along_edge(edge)
                    self.current_mode = 'circ'

    def detect_m_line(self):
        if self.lockout < 0:
            if self._find_collision(self.m_line):
                self.lockout = 2
                print('Crossed M-line')
                self.turn_to_goal()
                self.current_mode = 'goal'

    def _find_collision(self, edge):
        if edge['limit']['x'][0] < self.x < edge['limit']['x'][1] and edge['limit']['y'][0] < self.y < edge['limit']['y'][1]:
            x, y = self.position_history[-2]
            previous_side = edge['coefficients'][0] * x + edge['coefficients'][1] * y < edge['coefficients'][2]
            x, y = self.position_history[-1]
            current_side = edge['coefficients'][0] * x + edge['coefficients'][1] * y < edge['coefficients'][2]
            return previous_side != current_side

    def turn_along_edge(self, edge):
        if self.current_mode == 'circ':
            self._find_direction_to_turn(edge)
        else:
            self._rewind_position()
            self.heading = math.atan(-edge['coefficients'][0])

        print(np.rad2deg(self.heading))

    def circumnavigate(self, edge):
        self.current_mode = 'circ'
        pass

    def _find_direction_to_turn(self, edge):
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


class Anim:
    def __init__(self, data):
        self.x = []
        self.y = []
        self.data = data
        self.fig, self.ax = plt.subplots()

    def update_plot(self, frame):
        ind = next(counter)
        if ind > len(self.data) - 1:
            anim.event_source.stop()
            print('\nEnd of animation. Thanks for watching.')
            return
        self.x.append(self.data[ind][0])
        self.y.append(self.data[ind][1])
        plt.cla()
        self.ax.plot(self.x, self.y)
        self.ax.plot(self.x[-1], self.y[-1], marker='o')
        self.ax.plot([start_point[0], goal_point[0]], [start_point[1], goal_point[1]], marker='o')
        for edge in edges:
            self.ax.plot([point[0] for point in edge['points']], [point[1] for point in edge['points']], color='red')

        plt.title('Bug 1 Motion')
        plt.xlim([0, 780])
        plt.ylim([0, 400])


# Example usage:
if __name__ == "__main__":
    start_point = [75, 224]
    goal_point = [682, 223]
    edges = []
    contours = find_contours(image)
    for contour in contours:
        vertices = find_vertices(contour)
        for i in range(len(vertices) - 1):
            edges.append(find_line_equation(vertices[i], vertices[i + 1]))

    bug = Bug1(start_point, goal_point)
    print("Initial Position:", bug.get_position())
    bug.turn_to_goal()
    for i in range(0, 500):
        bug.move_forward(2)
        bug.detect_edges(edges)
        bug.detect_m_line()
    counter = count(0, 1)
    animation = Anim(bug.position_history)
    anim = FuncAnimation(animation.fig, animation.update_plot, blit=False, interval=10, save_count=1000)
    plt.show()
