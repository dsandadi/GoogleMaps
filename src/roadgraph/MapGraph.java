/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {
	// TODO: Add your member variables here in WEEK 3

	HashMap<Node, ArrayList<Node>> adjList;
	HashMap<GeographicPoint, Node> loc;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		adjList = new HashMap<>();
		loc = new HashMap<>();
		// TODO: Implement in this constructor in WEEK 3
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return adjList.size();
		// TODO: Implement this method in WEEK 3
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		// TODO: Implement this method in WEEK 3
		Set<GeographicPoint> a = new HashSet<>();

		for (Map.Entry<Node, ArrayList<Node>> e : adjList.entrySet())
			a.add(e.getKey().getLocation());

		return a;
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		int numEdges = 0;
		// TODO: Implement this method in WEEK 3
		for (Map.Entry<Node, ArrayList<Node>> e : adjList.entrySet())
			numEdges += e.getKey().getEdges().size();

		return numEdges;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		// TODO: Implement this method in WEEK 3
		if ((loc.containsKey(location)) || (location == null))
			return false;
		else {

			Node newNode = new Node(location);

			ArrayList<Node> neighbors = new ArrayList<>();

			adjList.put(newNode, neighbors);

			loc.put(location, newNode);

		}

		return true;

	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {

		Node start = loc.get(from);
		Node end = loc.get(to);

		// input validation.
		if (!(adjList.containsKey(start) || adjList.containsKey(end) || from == null || to == null || roadName == null
				|| roadType == null))
			throw new IllegalArgumentException();

		// create a new edge
		Edge e = new Edge(start, end, roadName, length, roadType);

		// initialize edges;
		start.getEdges().add(e);
		adjList.get(start).add(end);// adding to adjList.

	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {

		// TODO: Implement this method in WEEK 3

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());
		Node startNode = loc.get(start);
		Node goalNode = loc.get(goal);
		HashMap<Node, Node> parent = new HashMap<>();
		HashSet<Node> visited = new HashSet<>();

		Queue<Node> queue = new LinkedList<Node>();
		queue.add(startNode);
		parent.put(startNode, startNode);

		while (!queue.isEmpty()) {
			Node current = queue.remove();
			if (visited.contains(current))
				continue;
			nodeSearched.accept(current.getLocation());
			if (current == goalNode) {

				return extractPath(parent, current, startNode);
				// break;

			} else {
				for (Edge e : current.getEdges()) {
					// if(e.getEnd() == null) continue;
					if (!((queue.contains(e.getEnd())) && (visited.contains(e.getEnd())))) {
						if (!parent.containsKey(e.getEnd()))
							parent.put(e.getEnd(), current);
						queue.add(e.getEnd());
					}
				}
			}
			visited.add(current);
		}

		return null;
	}

	private List<GeographicPoint> extractPath(HashMap<Node, Node> parent, Node current, Node startNode) {
		List<GeographicPoint> a = new ArrayList<>();
		int i = 0;
		while (!parent.get(current).equals(current)) {
			a.add(current.getLocation());
			current = parent.get(current);
			System.out.println(current.toString());
			System.out.println();
			i++;

		}
		a.add(current.getLocation());
		Collections.sort(a, new MyComparator());

		return a;
	}

	private class MyComparator implements Comparator<GeographicPoint> {

		@Override
		public int compare(GeographicPoint arg0, GeographicPoint arg1) {
			// TODO Auto-generated method stub
			return -1;
		}

	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 4

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());
		Node startNode = loc.get(start);
		Node goalNode = loc.get(goal);

		Queue<Node> a = new PriorityQueue<>();

		return createAnswer(startNode, goalNode, nodeSearched, a);

	}

	private List<GeographicPoint> createAnswer(Node startNode, Node goalNode, Consumer<GeographicPoint> nodeSearched,
			Queue<Node> a) {
		HashSet<Node> visited = new HashSet<>();
		HashMap<Node, Node> parent = new HashMap<>();// ?Do I need it

		startNode.setCurrentDistance(0);
		a.add(startNode);
		int count = 0;
		while (!a.isEmpty()) {
			Node current = a.remove();
			count++;
			if (current == goalNode) {
				nodeSearched.accept(current.getLocation());	
				break;
				
			}
			if (!visited.contains(current)) {

				for (Edge e : current.getEdges()) {
					Node neigbor = e.getEnd();
					double updateDistance = current.getCurrentDistance() + e.getLength();
					if (((neigbor.getCurrentDistance() > updateDistance) && (!visited.contains(neigbor)))) {
						neigbor.setCurrentDistance(updateDistance);
						a.add(neigbor);
						parent.put(neigbor, current);
					}
				}
				visited.add(current);
				nodeSearched.accept(current.getLocation());
			}
		}
		
		System.out.println("Dijkstra count :"+ count);
		return findPath(parent, goalNode, startNode);
	}

	private List<GeographicPoint> createAnswerA(Node startNode, Node goalNode, Consumer<GeographicPoint> nodeSearched,
			Queue<Node> a) {
		HashSet<Node> visited = new HashSet<>();
		HashMap<Node, Node> parent = new HashMap<>();// ?Do I need it
		int count = 0;
		startNode.setCurrentDistance(0);
		startNode.setPrediction(0);
		a.add(startNode);

		while (!a.isEmpty()) {
			Node current = a.remove();
			count++;
			if (current == goalNode) {
				nodeSearched.accept(current.getLocation());
				break;
			}
			if (!visited.contains(current)) {

				for (Edge e : current.getEdges()) {
					Node neigbor = e.getEnd();

					double updatePrediction = neigbor.findDistance(goalNode);
					double updateDistance = current.getCurrentDistance() + e.getLength();
					double value1 = updatePrediction + updateDistance;
					double value2 = neigbor.getPrediction() + neigbor.getCurrentDistance();
					if (((value2 > value1) && (!visited.contains(neigbor)))) {
						neigbor.setCurrentDistance(updateDistance);
						neigbor.setPrediction(updatePrediction);
						a.add(neigbor);
						parent.put(neigbor, current);
					}
				}
				visited.add(current);
				nodeSearched.accept(current.getLocation());
			}
		}
		System.out.println("A*Search count :"+ count);
		return findPath(parent, goalNode, startNode);
	}

	private List<GeographicPoint> findPath(HashMap<Node, Node> parent, Node goalNode, Node startNode) {
		List<GeographicPoint> answer = new ArrayList<>();
		Node currentNode = goalNode;
		while (currentNode != startNode) {
			answer.add(currentNode.getLocation());
			currentNode = parent.get(currentNode);
		}
		answer.add(currentNode.getLocation());
		Collections.sort(answer, new MyComparator());
		return answer;
	}
	
		/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	private class MyAstarComparator implements Comparator<Node> {

		@Override
		public int compare(Node arg0, Node arg1) {
			// TODO Auto-generated method stub
			double value = (arg0.getCurrentDistance() + arg0.getPrediction())
					- (arg1.getCurrentDistance() + arg1.getPrediction());
			if (value > 0)
				return 1;
			else if (value < 0)
				return -1;
			else
				return 0;
		}

	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 4

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		Node startNode = loc.get(start);
		Node goalNode = loc.get(goal);

		Queue<Node> a = new PriorityQueue<>(new MyAstarComparator());

		return createAnswerA(startNode, goalNode, nodeSearched, a);
	}

	public static void main(String[] args) {
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		// You can use this method for testing.

		/*
		 * Here are some test cases you should try before you attempt the Week 3
		 * End of Week Quiz, EVEN IF you score 100% on the programming
		 * assignment.
		 */
		/*
		 * MapGraph simpleTestMap = new MapGraph();
		 * GraphLoader.loadRoadMap("data/testdata/simpletest.map",
		 * simpleTestMap);
		 * 
		 * GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		 * GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		 * 
		 * System.out.println(
		 * "Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5"
		 * ); List<GeographicPoint> testroute =
		 * simpleTestMap.dijkstra(testStart,testEnd); List<GeographicPoint>
		 * testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		 * 
		 * 
		 * MapGraph testMap = new MapGraph();
		 * GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		 * 
		 * // A very simple test using real data testStart = new
		 * GeographicPoint(32.869423, -117.220917); testEnd = new
		 * GeographicPoint(32.869255, -117.216927); System.out.println(
		 * "Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		 * testroute = testMap.dijkstra(testStart,testEnd); testroute2 =
		 * testMap.aStarSearch(testStart,testEnd);
		 * 
		 * 
		 * // A slightly more complex test using real data testStart = new
		 * GeographicPoint(32.8674388, -117.2190213); testEnd = new
		 * GeographicPoint(32.8697828, -117.2244506); System.out.println(
		 * "Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		 * testroute = testMap.dijkstra(testStart,testEnd); testroute2 =
		 * testMap.aStarSearch(testStart,testEnd);
		 */

		/* Use this code in Week 3 End of Week Quiz */

		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start, end);
		System.out.println("Dijkstran done");
		List<GeographicPoint> route2 = theMap.aStarSearch(start, end);

	}

}
