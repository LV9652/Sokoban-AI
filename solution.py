#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems


def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True


def heur_manhattan_distance(state):
  '''admissible sokoban puzzle heuristic: manhattan distance'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
  #We want an admissible heuristic, which is an optimistic heuristic.
  #It must never overestimate the cost to get from the current state to the goal.
  #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
  #When calculating distances, assume there are no obstacles on the grid.
  #You should implement this heuristic function exactly, even if it is tempting to improve it.
  #Your function should return a numeric value; this is the estimate of the distance to the goal.
  
  totManDist = 0

  for box in state.boxes:
    smallDist = state.width + state.height
    for goalBox in state.storage:
      dist = abs(goalBox[0] - box[0]) + abs(goalBox[1] - box[1])
      if dist < smallDist:
        smallDist = dist
    totManDist += smallDist 

  return totManDist


#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count


def heur_alternate(state):
  totManDist = 0
  goodAlongLeft, goodAlongRight, goodAlongBot, goodAlongTop = isAlongWall(state) 

  for box in state.boxes:
    if box not in state.storage:
      x, y = box
      top = (x, y+1)
      bottom = (x, y-1)
      left = (x-1, y)
      right = (x+1, y)

      #-----------------ALONG WALL BUT NO GOAL HERE-------------------#
      if x == 0 and goodAlongLeft == 0:
        return  float('inf')
      if x == state.width-1 and goodAlongRight == 0:
        return  float('inf')
      if y == 0 and goodAlongTop == 0:
        return  float('inf')
      if y == state.height-1 and goodAlongBot == 0:
        return  float('inf')

      #-----------------IS STUCK BETWEEN OBS OR WALL-------------------#
      if top in state.obstacles or bottom in state.obstacles or y == 0 or y == state.height-1:
        if  right in state.obstacles or left in state.obstacles or x == 0 or x == state.width-1:
          return float('inf')

      #-----------------IS STUCK BETWEEN WALL AND BOX-------------------#
      if x == 0 or x == state.width-1:
        if top in state.boxes or bottom in state.boxes:
          return float('inf')

      if y == 0 or  y == state.height-1:
        if left in state.boxes or right in state.boxes:
          return float('inf')

      smallDist = state.width + state.height
      for goalBox in state.storage:
        dist = abs(goalBox[0] - box[0]) + abs(goalBox[1] - box[1])
        if dist < smallDist:
          smallDist = dist
      totManDist += smallDist

  for someRobot in state.robots:
    robotDistSmall = state.width+state.height
    for box in state.boxes:
      robotDist = abs(someRobot[0] - box[0]) + abs(someRobot[1] - box[1])
      if robotDist < robotDistSmall:
        robotDistSmall = robotDist
    totManDist += robotDistSmall

  return totManDist


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0


def fval_function(sN, weight):
  """
  Provide a custom formula for f-value computation for Anytime Weighted A star.
  Returns the fval of the state contained in the sNode.

  @param sNode sN: A search node (containing a SokobanState)
  @param float weight: Weight given by Anytime Weighted A star
  @rtype: float
  """
  #Many searches will explore nodes (or states) that are ordered by their f-value.
  #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
  #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
  #The function must return a numeric f-value.
  #The value will determine your state's position on the Frontier list during a 'custom' search.
  #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function. 
  return sN.gval + weight * sN.hval


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  wrapped_fval_function = (lambda sN: fval_function(sN, weight))
  #init the seach
  se = SearchEngine('custom', 'full')
  se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_alternate, fval_function=wrapped_fval_function)
  costbound = (float('inf'), float('inf'), float('inf'))
  
  #perform first search
  startTime = os.times()[0]
  result = se.search(timebound, costbound)
  if result == False:
    return result
  timeLeft = timebound - (os.times()[0] - startTime)
  finalVal = result

  #if there is time left keep searching
  while timeLeft > 0:
    startTime = os.times()[0]
    result = se.search(timeLeft, costbound)
    timeLeft -= os.times()[0] - startTime

    if result != False:
      finalVal = result
      weight = weight * 0.6 #halve weight
      costbound = (result.gval, result.gval, result.gval)
      # costbound = (float("inf"), float("inf"), result.gval)
    else:
      #print (finalVal.gval)
      return finalVal

  #print (finalVal.gval)
  return finalVal


def anytime_gbfs(initial_state, heur_fn, timebound = 10):
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''

  se = SearchEngine('best_first', 'full')
  se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_alternate)
  costbound = (float('inf'), float('inf'), float('inf'))

  startTime = os.times()[0]
  result = se.search(timebound, costbound)
  if result == False:
    return result
  timeLeft = timebound - (os.times()[0] - startTime)
  finalVal = result

  while timeLeft > 0:
    startTime = os.times()[0]
    result = se.search(timeLeft, costbound)
    timeLeft -= os.times()[0] - startTime
    if result != False:
        finalVal = result
        costbound = (result.gval, result.gval, result.gval)
        # costbound = (result.gval, float("inf"), float("inf"))
    else:
      return finalVal

  return finalVal


def isAlongWall(state):
  #check if any goals are along the wall or in the coreners, save the ones that are
  goodAlongLeft = 0 
  goodAlongRight = 0
  goodAlongTop = 0
  goodAlongBot = 0

  for goalBox in state.storage:
    if goalBox not in state.boxes:
      x, y = goalBox
      if x == 0:
      #storage on lhs
        goodAlongLeft = 1 
      if x == state.width-1:
      #storage on rhs
        goodAlongRight = 1
      if y == 0:
      #up
        goodAlongBot = 1
      if y == state.height-1:
      #down
        goodAlongTop = 1

  return goodAlongLeft, goodAlongRight, goodAlongBot, goodAlongTop

