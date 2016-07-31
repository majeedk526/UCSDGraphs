/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.awt.print.Printable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.Stack;
import java.util.function.Consumer;

import javax.swing.JTable.PrintMode;

import com.sun.corba.se.impl.oa.poa.ActiveObjectMap.Key;

import java.util.Queue;

import geography.GeographicPoint;
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
	
	static Map<GeographicPoint, ArrayList<Route>> map;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		map = new HashMap<GeographicPoint, ArrayList<Route>>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return map.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		
		return map.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> keySet = map.keySet();
		
		int numEdges = 0;
		ArrayList<Route> connectedNodes;
		
		Set<Entry<GeographicPoint, ArrayList<Route>>> gpset = map.entrySet();
		
		for(Map.Entry<GeographicPoint, ArrayList<Route>> gp : gpset){
			connectedNodes = gp.getValue();
			if(connectedNodes != null){
				numEdges += connectedNodes.size();
			}
		}
		
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
		
		if(location != null){
			if(!map.containsKey(location)){
				map.put(location, new ArrayList<Route>());
				return true;
			}
		}
		
		return false;
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
		if(from == null || to==null || roadName == null || roadType==null || length==0d){
			throw new IllegalArgumentException("Check your arguements.");
		}
		map.get(from).add(new Route(from,to,roadName, roadType, length));
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public static List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
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
	public static List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		Queue<GeographicPoint> q = new LinkedList<GeographicPoint>();
		List<GeographicPoint> visited = new ArrayList<GeographicPoint>();
		
		List<VisitedNode> pathlist = new LinkedList<VisitedNode>(); // stores visited nodes
		List<GeographicPoint> shortestPath = null; // stores any possible path found
		List<GeographicPoint> bestPath = null; //new ArrayList<GeographicPoint>(); // stores the best short path
		
		List<GeographicPoint> neighbours = null;
		
		boolean found = false;		
		
		q.add(start);
		pathlist.add(new VisitedNode(start, null));
		
		
		while(!q.isEmpty()){
			GeographicPoint next = q.poll();
			VisitedNode parent = getParent(next,  pathlist);
			
		
			System.out.println("examining : " + next.x + " " + next.y);
			
			if(visited.contains(next)){
				continue;
			}
			
			nodeSearched.accept(next);
			
			visited.add(next);
			neighbours = getNeighbours(next);
			
			for(GeographicPoint n : neighbours){
				System.out.println("\tneighbours : " + n.x + " " + n.y);
				
				pathlist.add(new VisitedNode(n, parent));
				if(n.equals(goal)){	
					shortestPath = getShortestPath(pathlist, goal, start);
					for(GeographicPoint g : shortestPath){
						System.out.println("\t\tpath : " + g.x + " " + g.y);
					}
					if(bestPath==null){
						bestPath = shortestPath;
					}
					
					if(bestPath.size() > shortestPath.size()){
						bestPath = shortestPath;
					}
					
					found= true;
					break;
				}
			}
			q.addAll(neighbours);
		}
				
		if(found==true){
			return bestPath;
		} else {
			return null;
		}
		
	}
	
	private static VisitedNode getParent(GeographicPoint gp, List<VisitedNode> pathlist){
		
		for(VisitedNode vn : pathlist){
			if(vn.gpNode.equals(gp)){
				return vn;
			}
			
		}
		
		return null;
	}
	
	private static List<GeographicPoint> getShortestPath(List<VisitedNode> v, GeographicPoint goal, GeographicPoint start){
		List<GeographicPoint> sp = new ArrayList<GeographicPoint>();
		boolean found = false;
		Stack<GeographicPoint> stack = new Stack<GeographicPoint>();
		
		VisitedNode vn = getParent(goal, v);
		while(!found){
			stack.push(vn.gpNode);
			if(vn.gpNode.equals(start)){
				found = true;
			}
			vn = vn.parent;
		}
		
		while(!stack.isEmpty()){
			sp.add(stack.pop());
		}
		
		return sp;
	}
	
	private static List<GeographicPoint> getNeighbours(GeographicPoint location){
		List<Route> routes = map.get(location);
		List<GeographicPoint> gps = new ArrayList<GeographicPoint>();
		
		
		if(!routes.isEmpty()){
			for(Route r : routes){
				gps.add(r.to);
			}
		}
		
		return gps;
	}
	
	private static List<DNode> getRouteNeighbours(DNode curr){
		List<Route> routes = map.get(curr.r.to);
		List<DNode> dnodes = new ArrayList<DNode>();
		
		if(routes!=null){
			
		
		for(Route r : routes){
			dnodes.add(new DNode(r, curr.edgeCost + r.length, curr));
		}
		}
		
		return dnodes;
	}
	
	private static List<ANode> getRouteNeighbours(ANode curr, GeographicPoint start, GeographicPoint goal){
		List<Route> routes = map.get(curr.r.to);
		List<ANode> anodes = new ArrayList<ANode>();
		
		if(routes!=null){
		for(Route r : routes){
			anodes.add(new ANode(r, curr, getG(start,r.to), getH(goal, r.to)));
		}
		}
		return anodes;
	}
	
	
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public static List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
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
	public static List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		PriorityQueue<DNode> pq = new PriorityQueue<DNode>();
		Set<GeographicPoint> visitedSet = new HashSet<GeographicPoint>(); 
		List<DNode> pathlist = new ArrayList<DNode>();
		List<DNode> neighbours;
		List<GeographicPoint> shortPath = null;
		List<GeographicPoint> bestPath = null;
		double bestLength = 9999999999d;
		boolean found = false;
		int count=0;
		
		Route r = new Route(null, start,"","", 0d);
		DNode dn = new DNode(r,0d, null);
		pq.add(dn);
		
		
		while(!pq.isEmpty()){
			DNode curr = pq.poll();
			
			//System.out.println(curr.toString());
			//DNode parent = curr.parent;
			
			if(visitedSet.contains(curr.r.to)){continue;}
			count++;
			visitedSet.add(curr.r.to);
			nodeSearched.accept(curr.r.to);
			
			if(curr.r.to.equals(goal)){
				shortPath = getDijkstraShortestPath(pathlist, dn, curr);
				found = true;}
			
			neighbours = getRouteNeighbours(curr);
			for(DNode d : neighbours){
				
				//if(visitedSet.contains(d)){continue;}
				
				//System.out.println("neighbour : " + d.toString());
				pathlist.add(d);
				pq.add(d);
				
					/**for(GeographicPoint gp : shortPath){
						System.out.println("short path : " + gp.x + " " + gp.y + " lenght : " + d.edgeCost);
					}
					
					if(bestLength > d.edgeCost){
						bestLength = d.edgeCost;
						bestPath = shortPath;
					}**/
					
				}
				
			if(found){break;}	
			}
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		System.out.println("Dijkstra count : " + count);
		if(found){
			System.out.println("\n\n");
			for(GeographicPoint gp : shortPath){
				System.out.println("shortest path : " + gp.x + " " + gp.y + " lenght : ");
			}
			
			return shortPath;
		} else {
			return null;
		}
	}

	
	private static List<GeographicPoint> getDijkstraShortestPath(List<DNode> pathlist, DNode start, DNode goal){
		
		boolean found = false;
		List<GeographicPoint> sp = new ArrayList<GeographicPoint>();
		Stack<GeographicPoint> stack = new Stack<GeographicPoint>();
		
		DNode tmpNode = goal;
		
		while(!found){
			//System.out.println(tmpNode.r.to.x + " " + tmpNode.r.to.y);
			stack.push(tmpNode.r.to);
			if(tmpNode.equals(start)){
				found = true;
				break;
			}
			tmpNode =tmpNode.parent;
		}
		
		while(!stack.isEmpty()){
			sp.add(stack.pop());
			
		}
		
		if(found){return sp;}
		else { return null;}
		
	}
	
	
private static List<GeographicPoint> getDijkstraShortestPath(List<ANode> pathlist, ANode start, ANode goal){
		
		boolean found = false;
		List<GeographicPoint> sp = new ArrayList<GeographicPoint>();
		Stack<GeographicPoint> stack = new Stack<GeographicPoint>();
		
		ANode tmpNode = goal;
		
		while(!found){
			//System.out.println(tmpNode.r.to.x + " " + tmpNode.r.to.y);
			stack.push(tmpNode.r.to);
			if(tmpNode.equals(start)){
				found = true;
				break;
			}
			tmpNode =tmpNode.parent;
		}
		
		while(!stack.isEmpty()){
			sp.add(stack.pop());
			
		}
		
		if(found){return sp;}
		else { return null;}
		
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
		
		PriorityQueue<ANode> pq = new PriorityQueue<ANode>();
		Set<GeographicPoint> visitedSet = new HashSet<GeographicPoint>(); 
		List<ANode> pathlist = new ArrayList<ANode>();
		List<ANode> neighbours;
		List<GeographicPoint> shortPath = null;
		List<GeographicPoint> bestPath = null;
		double bestLength = 9999999999d;
		boolean found = false;
		int count=0;
		
		Route r = new Route(null, start,"","", 0d);
		ANode an = new ANode(r,null,getG(start, r.to), getH(goal, r.to));
		pq.add(an);
		
		
		while(!pq.isEmpty()){
			ANode curr = pq.poll();
			
			//System.out.println(curr.toString());
			
			if(visitedSet.contains(curr.r.to)){continue;}
			count++;
			visitedSet.add(curr.r.to);
			nodeSearched.accept(curr.r.to);
			
			if(curr.r.to.equals(goal)){
				shortPath = getDijkstraShortestPath(pathlist, an, curr);
				found = true;
			}	
			neighbours = getRouteNeighbours(curr, start, goal);
			for(ANode a : neighbours){
				
				//if(visitedSet.contains(a)){continue;}
				
				//System.out.println("neighbour : " + a.toString());
				pathlist.add(a);
				pq.add(a);
				
					/**if(bestLength > (a.G+a.H)){
						bestLength = (a.G + a.H);
						bestPath = shortPath;
					}**/
				
				}
			
			if(found){break;}
		}
		
		
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		System.out.println("astar count : " + count);
		if(found){
			System.out.println("\n\n");
			for(GeographicPoint gp : shortPath){
				//System.out.println("shortest path : " + gp.x + " " + gp.y + " lenght : ");
			}
			return shortPath;
			} 
		else {return null;}
	}
	
	private static double getH(GeographicPoint goal, GeographicPoint curr){
		return goal.distance(curr);
	}
	
	private static double getG(GeographicPoint start, GeographicPoint curr){
		return start.distance(curr);
	}

	
	private static void printMap(){
		Set<GeographicPoint> keySet = map.keySet();
		
		for(GeographicPoint gp : keySet){
			System.out.println("node : " + gp.x + " " + gp.y);
			List<Route> edges = map.get(gp);
			for(Route r : edges){
				System.out.println("\troute : " + r.to.x + " " + r.to.y);
			}
		}
		
	}
	
	public static void main(String[] args)
	{
		/**System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		**/
		
		
		//* Code for testing
		/**
		printMap();
		Set<GeographicPoint> nodes = map.keySet();
		GeographicPoint start = null;
		GeographicPoint goal = null;
		int i=0;
		for(GeographicPoint g : nodes){
			if(g.x == 7.0 && g.y == 3.0 ){start = g;
			}
			if(g.x==4 && g.y==-1){goal = g;
			}
		}
		
		
		System.out.println("start : " + start.x + " " + start.y);
		System.out.println("goal : " + goal.x + " " + goal.y);
	
		//bfs(start, goal);
		dijkstra(start, goal);**/
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/ucsd.map", theMap);
		System.out.println("DONE.");

		//GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		//GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		GeographicPoint start = new GeographicPoint(32.8709815, -117.2434254);
		GeographicPoint end = new GeographicPoint(32.8742087, -117.2381344);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
	}
	
}
