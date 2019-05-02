from pygame.locals import *
from math import sqrt


def dist(p1, p2):
    """ Computes the euclidean distance between two points.
    :param p1:
    :param p2:
    :return: distance
    """
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))


def point_rect_collision(point, rectangles):
    """ Checks whether a given point collides with a rectangle (array).
    :param point: [x,y]
    :param rectangles: eg: [pygame.Rect((40,10),(100,200))]
    :return:    True    Collision
                False   no Collision
    """
    for rect in rectangles:
        if rect.collidepoint(point):
            return True
    return False


def point_circle_collision(point, circle_center, radius):
    """ Checks whether a given point collides with a circle
    :param point:
    :param circle_center:
    :param radius:
    :return:    True    Collision
                False   no Collision
    """
    if dist(point, circle_center) <= radius:
        return True
    return False
