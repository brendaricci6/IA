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

    # Inicializa um conjunto para armazenar os estados já visitados e evitar ciclos e redundância.
    # Isso implementa a busca em grafo.
    visited = set()

    # Pega o estado inicial e o adiciona à fronteira com um caminho vazio.
    start_state = problem.getStartState()
    fringe.push((start_state, []))

    # Loop principal que continua enquanto houver nós na fronteira para explorar.
    while not fringe.isEmpty():
        # Remove o último nó adicionado da pilha (comportamento DFS).
        current_state, actions = fringe.pop()

        # Se o estado atual já foi visitado, pula para a próxima iteração para evitar reprocessamento.
        if current_state in visited:
            continue

        # Se o estado atual é o estado objetivo, encontramos a solução.
        # Retorna a lista de ações que nos levaram até aqui.
        if problem.isGoalState(current_state):
            return actions

        # Adiciona o estado atual ao conjunto de visitados para não explorá-lo novamente.
        visited.add(current_state)

        # Obtém os sucessores do estado atual.
        successors = problem.getSuccessors(current_state)

        # Itera sobre cada sucessor.
        for next_state, action, cost in successors:
            # Se o estado sucessor ainda não foi visitado, o adicionamos à fronteira.
            if next_state not in visited:
                # Cria o novo caminho adicionando a ação atual ao caminho existente.
                new_actions = actions + [action]
                # Adiciona o estado sucessor e seu novo caminho à pilha.
                fringe.push((next_state, new_actions))

    # Se a fronteira ficar vazia e o objetivo não for encontrado, retorna uma lista vazia.
    return []
    #util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # Inicializa a fronteira usando uma Fila (Queue) para o comportamento FIFO (First-In, First-Out) do BFS.
    # A fila armazenará tuplas contendo o estado e o caminho (lista de ações) para alcançá-lo.
    fringe = util.Queue()

    # Inicializa um conjunto para armazenar os estados já visitados e evitar ciclos e redundância.
    # Isso é crucial para a busca em grafo e para a eficiência.
    visited = set()

    # Pega o estado inicial e o adiciona à fronteira com um caminho vazio.
    start_state = problem.getStartState()
    fringe.push((start_state, []))
    
    # Adicionamos o estado inicial ao conjunto de visitados imediatamente, pois ele está na fronteira.
    # Isso evita adicionar o mesmo estado múltiplas vezes à fila se houver vários caminhos para ele
    # antes que ele seja expandido.
    visited.add(start_state)

    # Loop principal que continua enquanto houver nós na fronteira para explorar.
    while not fringe.isEmpty():
        # Remove o nó mais antigo da fila (comportamento BFS).
        current_state, actions = fringe.pop()

        # Se o estado atual é o estado objetivo, encontramos a solução.
        # Como o BFS explora nível a nível, este será o caminho mais curto em número de ações.
        if problem.isGoalState(current_state):
            return actions

        # Obtém os sucessores do estado atual.
        successors = problem.getSuccessors(current_state)

        # Itera sobre cada sucessor.
        for next_state, action, cost in successors:
            # Se o estado sucessor ainda não foi visitado, o processamos.
            if next_state not in visited:
                # Adiciona o sucessor ao conjunto de visitados para não o adicionar à fila novamente.
                visited.add(next_state)
                # Cria o novo caminho adicionando a ação atual ao caminho existente.
                new_actions = actions + [action]
                # Adiciona o estado sucessor e seu novo caminho à fila.
                fringe.push((next_state, new_actions))

    # Se a fronteira ficar vazia e o objetivo não for encontrado, retorna uma lista vazia.
    return []
    #util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    """Busca primeiro o nó de menor custo total."""
    # Inicializa a fronteira usando uma Fila de Prioridade (PriorityQueue).
    # Ela armazenará uma tupla (item, prioridade), onde o item é (estado, caminho)
    # e a prioridade é o custo acumulado para chegar àquele estado.
    fringe = util.PriorityQueue()

    # Inicializa um conjunto para armazenar os estados já visitados.
    # Como a Fila de Prioridade garante que a primeira vez que visitamos um nó
    # é pelo caminho de menor custo, não precisamos revisitá-lo.
    visited = set()

    # Pega o estado inicial e o adiciona à fronteira com um caminho vazio e custo 0.
    start_state = problem.getStartState()
    fringe.push((start_state, []), 0) # O item é (estado, ações), a prioridade é o custo.

    # Loop principal que continua enquanto houver nós na fronteira para explorar.
    while not fringe.isEmpty():
        # Remove o nó com a menor prioridade (menor custo) da fila.
        current_state, actions = fringe.pop()

        # Se já expandimos este estado, pulamos.
        # Isso é crucial para a eficiência.
        if current_state in visited:
            continue

        # Adiciona o estado atual ao conjunto de visitados para não expandi-lo novamente.
        visited.add(current_state)

        # Se o estado atual é o estado objetivo, encontramos a solução de menor custo.
        if problem.isGoalState(current_state):
            return actions

        # Obtém os sucessores do estado atual.
        successors = problem.getSuccessors(current_state)

        # Itera sobre cada sucessor.
        for next_state, action, cost in successors:
            # Se o estado sucessor ainda não foi visitado, o adicionamos à fronteira.
            if next_state not in visited:
                # Cria o novo caminho e calcula seu custo total.
                new_actions = actions + [action]
                new_cost = problem.getCostOfActions(new_actions)
                # Adiciona o sucessor à fila de prioridade com seu novo custo.
                fringe.push((next_state, new_actions), new_cost)

    # Se a fronteira ficar vazia e o objetivo não for encontrado, retorna uma lista vazia.
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
     # Inicializa a fronteira usando uma Fila de Prioridade.
    # O item armazenado será (estado, ações).
    # A prioridade será o custo_f = custo_g (custo do caminho) + custo_h (heurística).
    fringe = util.PriorityQueue()

    # Este dicionário substitui o conjunto 'visited'. Ele armazena o menor custo (g)
    # encontrado até agora para chegar a cada estado. Isso nos permite atualizar
    # um nó se encontrarmos um caminho melhor para ele.
    closed = {}

    # Configuração inicial para o estado de partida
    start_state = problem.getStartState()
    start_g_cost = 0
    
    # Adiciona o nó inicial à fronteira
    fringe.push((start_state, [], start_g_cost), start_g_cost + heuristic(start_state, problem))

    while not fringe.isEmpty():
        # Pega o nó com o menor custo f
        current_state, actions, g_cost = fringe.pop()

        # Se já encontramos um caminho para este estado e o caminho que acabamos de
        # retirar é mais caro, nós o ignoramos e continuamos.
        if current_state in closed and g_cost >= closed[current_state]:
            continue

        # Marca o estado como "fechado" com seu custo ótimo até agora.
        closed[current_state] = g_cost

        # Se é o objetivo, encontramos o caminho ótimo.
        if problem.isGoalState(current_state):
            return actions

        # Gera os sucessores
        for next_state, action, step_cost in problem.getSuccessors(current_state):
            new_g_cost = g_cost + step_cost
            
            # Se não vimos este nó antes, ou se encontramos um caminho mais barato para ele,
            # nós o adicionamos à fronteira para ser explorado.
            if next_state not in closed or new_g_cost < closed[next_state]:
                new_h_cost = heuristic(next_state, problem)
                new_f_cost = new_g_cost + new_h_cost
                new_actions = actions + [action]
                fringe.push((next_state, new_actions, new_g_cost), new_f_cost)

    # Retorna uma lista vazia se nenhum caminho for encontrado
    return []

    #util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
