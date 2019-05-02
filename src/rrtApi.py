#!/usr/bin/env python

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
from rrtNode import *
from collisionHandler import *
import enum


BACKGROUND_COLOR = [255, 255, 255]
OBSTACLE_COLOR = [100, 100, 100]
FINAL_PATH_COLOR = [0, 255, 100]
BRANCH_COLOR = [0, 0, 0]
START_POINT_COLOR = [0, 0, 255]
TARGET_POINT_COLOR = [255, 0, 0]
WINDOW_TITLE = 'Rapidly Exploring Random Tree'
MAX_NUM_OF_NODES = 10000
MAX_NUM_OF_SEARCHES = 100
GOAL_RADIUS = 15
EPSILON = 7.0


class States(enum.Enum):
    """ Enum for feasible processing states."""
    ERROR = 0
    INIT = 1
    TARGET_REACHED = 2
    BUILD_TREE = 3


class RapidlyExploringRandomTree:
    """ Rapidly Exploring Random Tree Algorithm. """

    def __init__(self, window_size, start_point, target_point, obstacles=[]):
        """ Initialization method.
        :param window_size:     [x,y]
        :param start_point:     [x,y]
        :param target_point:    [x,y]
        :param obstacles:       eg: [pygame.Rect((40,10),(100,200))]
        """
        pygame.init()
        self.fps_clock = pygame.time.Clock()
        self.window_size_x = window_size[0]
        self.window_size_y = window_size[1]
        self.start_point = start_point
        self.target_point = target_point
        self.target_node = None
        self.obstacles = obstacles
        self.screen = None
        self.count = 0
        self.nodes = [Node(self.start_point, None)]
        self.random_feasible_point = []
        self.parent_node = None
        self.state = States.INIT

    @staticmethod
    def step_from_to(p1, p2):
        """ Computes the next point, when traveling form 1 to 2
        with respect to a certain minimum quantization step EPSILON.
        Is the distance of EPSILON not met, then a new point is computed,
        which is not collinear with p1 and p2 in order to provide some randomness.
        :param p1:
        :param p2:
        :return: next point
        """
        if dist(p1, p2) < EPSILON:
            return p2
        else:
            theta = atan2(p2[1]-p1[1], p2[0]-p1[0])
            return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

    def get_random(self):
        """ Returns a random point on the map.
        :return: random point
        """
        return random.random() * self.window_size_x,\
               random.random() * self.window_size_y

    def get_feasible_point(self):
        """ Searches randomly for point, which not collide with any obstacles.
        :return: point
        """
        while True:
            point = self.get_random()
            if not point_rect_collision(point, self.obstacles):
                return point

    def draw_obstacles(self):
        """ Draws the pre-defined obstacles.
        """
        if not self.obstacles.__len__() == 0:
            for rect in self.obstacles:
                pygame.draw.rect(self.screen, OBSTACLE_COLOR, rect)

    def draw(self):
        """ Draws the whole board.
        """
        self.screen = pygame.display.set_mode([self.window_size_x, self.window_size_y])
        self.screen.fill(BACKGROUND_COLOR)
        pygame.display.set_caption(WINDOW_TITLE)
        self.draw_obstacles()
        self.draw_start_point()
        self.draw_target_point()

    def draw_start_point(self):
        """ Draws the start point.
        """
        pygame.draw.circle(self.screen, START_POINT_COLOR, self.start_point, GOAL_RADIUS)

    def draw_target_point(self):
        """ Draws the target point.
        """
        pygame.draw.circle(self.screen, TARGET_POINT_COLOR, self.target_point, GOAL_RADIUS)

    def draw_branch(self, start_node, target_node):
        """ Draws a line from the start node to the target node.
        :param start_node:
        :param target_node:
        """
        pygame.draw.line(self.screen, BRANCH_COLOR, start_node.point, target_node.point)

    def path_traceback(self, target_node):
        """ Call this, when the target has been reached.
        This will highlight the final path.
        :param: target_node
        """
        current_node = target_node.parent
        while current_node.parent is not None:
            pygame.draw.line(self.screen, FINAL_PATH_COLOR, current_node.point, current_node.parent.point)
            current_node = current_node.parent

    def find_nearest_vertex(self):
        """ Searches for the nearest feasible vertex.
        :return:    True    search successful
                    False   search not successful
        """
        self.random_feasible_point = self.get_feasible_point()
        self.parent_node = self.nodes[0]
        return_state = False
        for node in self.nodes:
            # check to see if this vertex is closer than the previously selected closest
            if dist(node.point, self.random_feasible_point) <= dist(self.parent_node.point, self.random_feasible_point):
                new_point = self.step_from_to(node.point, self.random_feasible_point)
                # check if a collision would occur with the newly selected vertex
                if not point_rect_collision(new_point, self.obstacles):
                    self.parent_node = node
                    return_state = True
        return return_state

    def find_next_node(self):
        """ Searches for the next feasible node.
        :return:    True    found next node
                    False   no node found
        """
        for i in range(MAX_NUM_OF_SEARCHES):
            if self.find_nearest_vertex():
                return True
        return False

    def check_if_target_reached(self, node):
        """ Checks if the target is reachable.
        :param node:
        """
        if point_circle_collision(node.point, self.target_point, GOAL_RADIUS):
            self.state = States.TARGET_REACHED
            self.target_node = self.nodes[len(self.nodes) - 1]

    def build_tree(self):
        """ Builds the tree.
        """
        self.count = self.count + 1
        if self.count < MAX_NUM_OF_NODES:
            if not self.find_next_node():
                print("ERROR: Given problem not solvable!")
            new_point = self.step_from_to(self.parent_node.point, self.random_feasible_point)
            new_node = Node(new_point, self.parent_node)
            self.nodes.append(new_node)
            self.draw_branch(self.parent_node, new_node)
            self.check_if_target_reached(new_node)
            if self.count % 100 == 0:
                print("node: {0}".format(self.count))
        else:
            print("ERROR: Reached max number of nodes!")

    def handle_process_states(self):
        """ This methods handles the process states
        and starts appropriated methods.
        """
        if self.state == States.INIT:
            self.fps_clock.tick(10)
        elif self.state == States.TARGET_REACHED:
            self.path_traceback(self.target_node)
            self.state = States.INIT
        elif self.state == States.BUILD_TREE:
            self.build_tree()
        elif self.state == States.ERROR:
            print("ERROR: Error state reached!")
        else:
            print("ERROR: Invalid process state!")

    def run(self):
        """ Starts the algorithm.
        """
        self.draw()
        print("Hit SPACE to START ...")
        print("Hit ESC to QUIT ...")
        while True:
            self.handle_process_states()
            pygame.display.update()
            self.check_for_user_input()
            self.fps_clock.tick(10000)

    def check_for_user_input(self):
        """ Checks for certain key inputs
        and starts appropriated consequences.
        """
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == KEYUP and e.key == K_SPACE:
                self.state = States.BUILD_TREE

