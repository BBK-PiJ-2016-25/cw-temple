package student;

import game.EscapeState;
import game.ExplorationState;
import game.Node;
import game.NodeStatus;

import java.util.*;

public class Explorer {

  private Map<Long, Integer> visited = new TreeMap<>();
  private Map<Long, Vertex> escapeVisited = new HashMap<Long, Vertex>();

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
      long nextLocation = getNextLocation(state);
      if (!visited.containsKey(nextLocation)) {
        visited.put(nextLocation, 1);
      } else {
        visited.put(nextLocation, visited.get(nextLocation) + 1);
      }
      state.moveTo(nextLocation);
    }
    return;
  }

  private Long getNextLocation(ExplorationState state) {
    Comparator<NodeStatus> comparatorVisited =
            Comparator.comparingInt(a -> visited.getOrDefault(a.getId(), 0));
    Comparator<NodeStatus> comparatorDistance =
            Comparator.naturalOrder();

    final Object[] children =
                    state.getNeighbours()
                    .stream()
                    .sorted(comparatorVisited.thenComparing(comparatorDistance))
                    .map(a -> a.getId())
                    .toArray();
    return (Long) children[0];
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
        Node startNode = state.getCurrentNode();
        Node exitNode = state.getExit();

        Map<Node, Boolean> visited = new HashMap<Node, Boolean>();

        Map<Node, Node> previous = new HashMap<Node, Node>();

        List<Node> directions = new LinkedList();
        Queue<Node> q = new LinkedList();
        Node current = startNode;

        q.add(current);
        visited.put(current, true);
        while(!q.isEmpty()){
          current = q.remove();
          if (current.equals(exitNode)){
            break;
          }else{
            for(Node node : current.getNeighbours()){
              if(!visited.containsKey(node)){
                q.add(node);
                visited.put(node, true);
                previous.put(node, current);
              }
            }
          }
        }
        for(Node node = exitNode; node != null; node = previous.get(node)) {
          directions.add(node);
        }
        Collections.reverse(directions);
        System.out.println(directions);
        directions.remove(0);
        directions.stream().forEach(a -> state.moveTo(a));
      }
}
