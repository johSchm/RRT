import numpy as np
import matplotlib.pyplot as plt
import rrtApi
import pygame


def main():
    obstacles = [pygame.Rect((40, 10), (100, 200)),
                 pygame.Rect((300, 300), (450, 450))]
    rrt = rrtApi.RapidlyExploringRandomTree([800, 800], [10, 10], [500, 100], obstacles)
    rrt.run()


if __name__ == '__main__':
    main()
