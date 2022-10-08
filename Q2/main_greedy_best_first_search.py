from missionaryCannibal import *
from search import greedy_best_first_graph_search

if __name__ == '__main__':
    p = MissionaryCannibal()
    greedy_best_first_graph_search(p, p.h)
