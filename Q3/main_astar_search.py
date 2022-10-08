from usMap import *
from search import astar_search

if __name__ == '__main__':
    p = GraphProblem('Seattle', 'Dallas', roadMap)
    astar_search(p)
