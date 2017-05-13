package student;

import game.EscapeState;
import game.ExplorationState;
import game.Node;
import game.NodeStatus;

import java.util.*;
import java.util.stream.Collectors;

public class Explorer {

  /**
   * A map holding the ids of nodes that have been visited in the current phase
   * (cleared prior to escape phase).
   * For the explore phase, these are mapped against an integer representing the
   * number of times the node has been visited.
   */
  private Map<Long, Integer> visited = new TreeMap<>();

  /**
   * Explore the cavern, trying to find the orb in as few steps as possible.
   * Once you find the orb, you must return from the function in order to pick
   * it up. If you continue to move after finding the orb rather
   * than returning, it will not count.
   * If you return from this function while not standing on top of the orb,
   * it will count as a failure.
   * <p>
   * There is no limit to how many steps you can take, but you will receive
   * a score bonus multiplier for finding the orb in fewer steps.
   * <p>
   * At every step, you only know your current tile's ID and the ID of all
   * open neighbor tiles, as well as the distance to the orb at each of these tiles
   * (ignoring walls and obstacles).
   * <p>
   * To get information about the current state, use functions
   * getCurrentLocation(),
   * getNeighbours(), and
   * getDistanceToTarget()
   * in ExplorationState.
   * You know you are standing on the orb when getDistanceToTarget() is 0.
   * <p>
   * Use function moveTo(long id) in ExplorationState to move to a neighboring
   * tile by its ID. Doing this will change state to reflect your new position.
   * <p>
   * A suggested first implementation that will always find the orb, but likely won't
   * receive a large bonus multiplier, is a depth-first search.
   *
   * @param state the information available at the current state
   */
  public void explore(ExplorationState state) {
    visited.put(state.getCurrentLocation(),1);
    while (state.getDistanceToTarget() != 0) {
      // Fetches the next suggested location
      long nextLocation = getNextLocation(state);
      /* Logs the location as visited once if it has not yet been visited, or increasing the visited
      counter by one if it has */
      if (!visited.containsKey(nextLocation)) {
        visited.put(nextLocation, 1);
      } else {
        visited.put(nextLocation, visited.get(nextLocation) + 1);
      }
      // Moves to the next location
      state.moveTo(nextLocation);
    }
  }

  /**
   * Returns the id for the next node to be visited according to an optimisation
   * algorithm based on the number of times the node has been visited and its distance
   * from the target.
   * @param state the information available at the current state.
   * @return the id of the node suggested for visiting next.
   */
  private Long getNextLocation(ExplorationState state) {
    // Comparator by the number of times the node has been visited.
    Comparator<NodeStatus> comparatorVisited =
            Comparator.comparingInt(a -> visited.getOrDefault(a.getId(), 0));
    // Comparator by the distance of the node from the target (exit).
    Comparator<NodeStatus> comparatorDistance =
            Comparator.naturalOrder();

    return state.getNeighbours()
            .stream()
            .sorted(comparatorVisited.thenComparing(comparatorDistance))
            .map(a -> a.getId())
            .findFirst()
            .get();
  }

    /**
   * Escape from the cavern before the ceiling collapses, trying to collect as much
   * gold as possible along the way. Your solution must ALWAYS escape before time runs
   * out, and this should be prioritized above collecting gold.
   * <p>
   * You now have access to the entire underlying graph, which can be accessed through EscapeState.
   * getCurrentNode() and getExit() will return you Node objects of interest, and getVertices()
   * will return a collection of all nodes on the graph.
   * <p>
   * Note that time is measured entirely in the number of steps taken, and for each step
   * the time remaining is decremented by the weight of the edge taken. You can use
   * getTimeRemaining() to get the time still remaining, pickUpGold() to pick up any gold
   * on your current tile (this will fail if no such gold exists), and moveTo() to move
   * to a destination node adjacent to your current node.
   * <p>
   * You must return from this function while standing at the exit. Failing to do so before time
   * runs out or returning from the wrong location will be considered a failed run.
   * <p>
   * You will always have enough time to escape using the shortest path from the starting
   * position to the exit, although this will not collect much gold.
   *
   * @param state the information available at the current state
   */
  public void escape(EscapeState state) {
    // Clears the map of visited nodes from the escape phase
    visited.clear();
    List<Node> directions = new LinkedList<>();
    // Loops through all nodes in the graph ordered by the amount of gold on their associated tile
    for (Node node: getNodesOrderedByAmountOfGold(state)) {
      if (!visited.containsKey(node.getId()) && // If the node has not yet been visited on the way to another node...
              state.getTimeRemaining() >  // ... and the time remaining is greater than...
                      findWeightedLengthOfPath(state.getCurrentNode(),
                              findShortestPath(state.getCurrentNode(), node))/*... the length of the path to the node
                               under consideration... */
                              + findWeightedLengthOfPath(node, findShortestPath(node, state.getExit())))/*... plus the length of the
                path from that node to the exit*/ {
        //Moves to the node under consideration, picking up any gold en rout
        findShortestPath(state.getCurrentNode(), node).forEach(a -> moveAndPick(state, a));
      }
    }
    // Takes the shortest route to the exit, picking up any gold en route
    directions = findShortestPathToExit(state);
    directions.forEach(a -> moveAndPick(state, a));
  }

  /**
   * Moves to a specified node, picking up any gold encountered on the way, and marking each node as visited
   * @param state the information available at the current state
   * @param node the node to be moved to
   */
  public void moveAndPick(EscapeState state, Node node) {
    state.moveTo(node);
    visited.put(node.getId(), 0);
    if(state.getCurrentNode().getTile().getGold() > 0) {
      state.pickUpGold();
    }
  }

  /**
   * Returns all nodes in the graph ordered by the amount of gold on their associated tile
   * @param state the information available at the current state
   * @return a list of all nodes in the graph ordered by the amount of gold on their associated tile
   */
  public List<Node> getNodesOrderedByAmountOfGold(EscapeState state) {
    Comparator<Node> comparatorGold =
            (a, b) -> Integer.compare( b.getTile().getGold(), a.getTile().getGold());
    return state.getVertices().stream().sorted(comparatorGold).collect(Collectors.toList());
  }

  /**
   * Finds the true length of a path given the weights of all its edges
   * @param startingNode the node on which the path starts
   * @param directions directions to a target node
   * @return the sum of all edges on the route
   */
  public int findWeightedLengthOfPath(Node startingNode, List<Node> directions) {
    int distance = 0;
    // We return zero if there are no nodes in the list of directions
    if (directions.size() == 0) {
      return 0;
    } else {
      distance = startingNode.getEdge(directions.get(0)).length();
      for (int i = 0; i < directions.size() - 1; i++) {
        distance += directions.get(i).getEdge(directions.get(i + 1)).length();
      }
    }
    return distance;
  }

  /**
   * Using a Breadth First Search algorithm, returns the shortest path between two specified nodes
   * @param startNode the node at which the path starts
   * @param exitNode the node at which the path ends
   * @return a list of nodes in the order to be traversed to comprise the shortest path
   */
  public List<Node> findShortestPath(Node startNode, Node exitNode) {
    // A map indicating whether a node has been visited by the algorithm
    Map<Node, Boolean> visited = new HashMap<Node, Boolean>();
    // A map storing each node visited as well as the node previously visited
    Map<Node, Node> previous = new HashMap<Node, Node>();

    // The directions to be returned
    List<Node> directions = new LinkedList();
    // A LIFO structure enabling execution of the algorithm
    Queue<Node> queue = new LinkedList();
    Node current = startNode;

    queue.add(current);
    visited.put(current, true);
    while (!queue.isEmpty()) {
      current = queue.remove();
      if (current.equals(exitNode)) {
        break;
      } else {
        for (Node node : current.getNeighbours()) {
          if (!visited.containsKey(node)) {
            queue.add(node);
            visited.put(node, true);
            previous.put(node, current);
          }
        }
      }
    }
    // Constructing shorted path directions using the map of nodes visited previously to each node
    for (Node node = exitNode; node != null; node = previous.get(node)) {
      directions.add(node);
    }
    // Reversing to give start-finish order, as opposed to finish-start
    Collections.reverse(directions);
    // Removing the starting node from the directions
    directions.remove(0);
    return directions;
  }

  /**
   * A helper method returning the shortest path from the current node to the exit for a given state
   * @param state the information available at the current state
   * @return the shortest path from current node to exit
   */
  public List<Node> findShortestPathToExit(EscapeState state) {
    return findShortestPath(state.getCurrentNode(), state.getExit());
  }
}

