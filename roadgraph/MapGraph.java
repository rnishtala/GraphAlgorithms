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
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;
import java.util.function.Consumer;

import application.MarkerManager;
import application.RouteVisualization;

import geography.GeographicPoint;
import roadgraph.GraphNode;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private Map<GraphNode,ArrayList<GraphNode>> adjListMap;
	//private Queue<GeographicPoint> bfsQueue;
	private int numVertices;
	private int numEdges;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		adjListMap = new HashMap<GraphNode,ArrayList<GraphNode>>();
		numVertices = 0;
		numEdges = 0;
		//bfsQueue = new LinkedList<GeographicPoint>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> points = new HashSet<GeographicPoint>();
		for(GraphNode n : adjListMap.keySet()){
			points.add(n.getLocation());
		}
		return points;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		
		GraphNode vertex = new GraphNode(location);
		if(adjListMap.containsKey(vertex)){
			return false;
		}
		numVertices++;
		// System.out.println("Adding vertex "+v);
		ArrayList<GraphNode> neighbors = new ArrayList<GraphNode>();
		adjListMap.put(vertex,  neighbors);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		GraphEdge edge = null;
		GraphNode sourceNode = new GraphNode(from);
		GraphNode destNode = new GraphNode(to);
		ArrayList<GraphNode> neighbors = new ArrayList<GraphNode>();
		if(adjListMap.containsKey(sourceNode) && adjListMap.containsKey(destNode)){
			neighbors = adjListMap.get(sourceNode);
			neighbors.add(destNode);
			adjListMap.put(sourceNode, neighbors);
			edge = new GraphEdge(from,to,roadName,roadType,length);
			
			numEdges++;
		}
		else{
			System.out.println("Graph does not contain that intersection");
		}
		
	}
	
	public List<GraphNode> getNeighbors(GraphNode node){
		
		if(adjListMap.containsKey(node)){
			return adjListMap.get(node);
		}
		
		return null;

	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> System.out.println("BFS Node:\n"+x);
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		GraphNode sNode = new GraphNode(start);
		GraphNode dNode = new GraphNode(goal);
		Queue<GraphNode> bfsQueue = new LinkedList<GraphNode>();
		HashSet<GraphNode> visited = new HashSet<GraphNode>();
		HashMap<GraphNode,GraphNode> parentMap = new HashMap<GraphNode,GraphNode>();
		GraphNode curr = null;
	    List<GeographicPoint> bfsPath = new LinkedList<GeographicPoint>();
		bfsQueue.add(sNode);
		while(!bfsQueue.isEmpty()){
			curr = bfsQueue.remove();
			if(curr.equals(goal))
				return unwindParents(parentMap,start,goal) ;
			for(GraphNode n : getNeighbors(curr)){
				if(!visited.contains(n)){
					visited.add(n);
					nodeSearched.accept(n.getLocation());
					parentMap.put(n, curr);
					bfsQueue.add(n);
				}
			}
		}
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		return null;
	}
	
	private List<GeographicPoint> unwindParents(Map<GraphNode,GraphNode> parents, 
			GraphNode start, GraphNode goal) {
		//linked list allows simple and efficient addFirst() method
		LinkedList<GraphNode> unwound = new LinkedList<>();
		unwound.addFirst(goal);
		GraphNode curr = goal;
		//iterate backwards through map using each value as the next key
		while (!curr.equals(start)) {
			GraphNode next = parents.get(curr);
			unwound.addFirst(next.);
			curr = next;
		}
		return unwound;
	}
	
	public void printGraph(){
		
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		GraphLoader.createIntersectionsFile("data/testdata/simpletest.map",
                "data/intersections/simpletest.intersections");
		
		
		// You can use this method for testing.  
		
		GeographicPoint start = new GeographicPoint(8.0, -1.0);
		GeographicPoint end = new GeographicPoint(1.0, 1.0);
		
		List<GeographicPoint> path = theMap.bfs(start, end);
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
