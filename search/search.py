# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from game import Directions
from typing import List

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()




def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    fringe = util.Stack()

    #conjunto para armazenar os estados já visitados e evitar ciclos e redundância
    visited = set()

    #adiciona o estado inicial na fronteira com um caminho vazio
    start_state = problem.getStartState()
    fringe.push((start_state, []))

    #continua enquanto houver nós na fronteira para explorar
    while not fringe.isEmpty():
        #remove o último nó adicionado da pilha
        current_state, actions = fringe.pop()

        #se o estado atual já foi visitado, pula para a próxima iteração
        if current_state in visited:
            continue

        # ae o estado atual é o estado objetivo ent retorna a lista de ações q levam até aqui.
        if problem.isGoalState(current_state):
            return actions

        # adiciona o estado atual ao conjunto de visitados
        visited.add(current_state)

        #acha os sucessores do estado atual
        successors = problem.getSuccessors(current_state)

        # Iterações sobre cada sucessor,
        for next_state, action, cost in successors:
            #se o estado sucessor ainda não foi visitado
            #adiciona à fronteira
            if next_state not in visited:
                #criação de umnovo caminho e adicionado a ação atual ao caminho existente
                new_actions = actions + [action]
                #adiciona o estado sucessor e seu novo caminho na pilha
                fringe.push((next_state, new_actions))

    #se a fronteira ficar vazia e o objetivo não for encontrado
    #retorna uma lista vazia.
    return []
    #util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    #inicializa a fronteira usando uma fila (FIFO
    fringe = util.Queue()

    #inicializa um conjunto para armazenar os estados já visitados e evitar ciclos e redundância
    visited = set()

    #adiciona o estado inicial a fronteira com um caminho vazio
    start_state = problem.getStartState()
    fringe.push((start_state, []))
    
    #adiciona o estado inicial ao conjunto de visitados imediatamente
    visited.add(start_state)

    #enquanto houver nós na fronteira para explorar
    while not fringe.isEmpty():
        #remoção do nó mais antigo da fila
        current_state, actions = fringe.pop()

        #verifica se o estado atual é o estado objetivo
        if problem.isGoalState(current_state):
            return actions

        #obtem os sucessores do estado atual
        successors = problem.getSuccessors(current_state)

        #itera cada sucessor
        for next_state, action, cost in successors:
            #se ele não foi visitado, ele é processado
            if next_state not in visited:
                #adiciona o sucessor ao conjunto de visitados
                visited.add(next_state)
                #cria o novo caminho adicionando a ação atual ao caminho existente
                new_actions = actions + [action]
                # Adiciona o estado sucessor e seu novo caminho
                fringe.push((next_state, new_actions))

    # Se a fronteira ficar vazia e o objetivo não for encontrado, retorna uma lista vazia.
    return []
    #util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    """Busca primeiro o nó de menor custo total."""
    # Inicializa a fronteira usando uma Fila de Prioridade
    fringe = util.PriorityQueue()

    # Inicializa um conjunto para armazenar os estados visitados
    visited = set()

    # Adiciona o estado inicial e o adiciona na fronteira
    start_state = problem.getStartState()
    fringe.push((start_state, []), 0)

    #enquanto houver nós na fronteira para explorar
    while not fringe.isEmpty():
        # remove o nó com a menor prioridade
        current_state, actions = fringe.pop()

        if current_state in visited:
            continue

        #add o estado atual ao conjunto de visitaos
        visited.add(current_state)

        #se o estado atual é o estado objetivo
        if problem.isGoalState(current_state):
            #retorna a solução
            return actions

        #sucessores do estado atual
        successors = problem.getSuccessors(current_state)

        #itera para cada sucessor.
        for next_state, action, cost in successors:
            if next_state not in visited:
                new_actions = actions + [action]
                new_cost = problem.getCostOfActions(new_actions)
                fringe.push((next_state, new_actions), new_cost)

    #se a fronteira ficar vazia e o objetivo não for encontrado
    return []
    #util.raiseNotDefined()

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
     #fila de Prioridade
    fringe = util.PriorityQueue()

    closed = {}

    #conf inicial do estado de partida
    start_state = problem.getStartState()
    start_g_cost = 0
    
    #add o nó inicial na fronteira
    fringe.push((start_state, [], start_g_cost), start_g_cost + heuristic(start_state, problem))

    while not fringe.isEmpty():
        #pega o nó com o menor custo f
        current_state, actions, g_cost = fringe.pop()

        # Se um caminho para este estado jpa foi encontrado e o caminho recente encontrado e o caminho encontrado é mais caro].
        if current_state in closed and g_cost >= closed[current_state]:
            #so ignora
            continue

        #marca o estado com o custo ótimo
        closed[current_state] = g_cost

        #se é o objetivo ent o caminhoé  ótimo
        if problem.isGoalState(current_state):
            return actions

        #gera sucessores
        for next_state, action, step_cost in problem.getSuccessors(current_state):
            new_g_cost = g_cost + step_cost
            
            if next_state not in closed or new_g_cost < closed[next_state]:
                new_h_cost = heuristic(next_state, problem)
                new_f_cost = new_g_cost + new_h_cost
                new_actions = actions + [action]
                fringe.push((next_state, new_actions, new_g_cost), new_f_cost)

    #retorna uma lista vazia se nenhum caminho for encontrado
    return []

    #util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
