package graphPackage;



import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

import graphPackage.Graph.Node;
import me.jjfoley.gfx.IntPoint;
import me.jjfoley.gfx.TextBox;


public class Graph {
	/**
	 * the graph object alone  is super simple! it saves nodes and edges, and they store everything else
	 */
	ArrayList<Node> nodes;
	ArrayList<Edge> edges;
	
	public Graph() {
		this.nodes = new ArrayList<>();
		this.edges = new ArrayList<>();
	}
	
	/**
	 * this class stores all of the edges attached to it
	 * as well as the nodes those edges connect it to
	 * it stores its own location as a point
	 * stores its own name, which in the GUI right now is always just "node"
	 * for the purpose of breadth/depth searches, this node has a visited boolean so it knows if it has already been visited during a graph traversal
	 * it can also, for djikstra/breadth/depth, record the previous node so that you can trace a path between any two nodes by continually going to the previous node
	 */
	public static class Node {
		boolean deadEnd;
		IntPoint position;
		ArrayList<Node> neighbors; 
		ArrayList<Edge> edges; 
		String name;
		double distance;
		boolean visited = false;
		Node previous; //the previous node in a path, useful for djikstra
		public Node(String name, IntPoint position) {
			this.name = name;
			this.position = position;
			this.edges = new ArrayList<>();
			this.neighbors = new ArrayList<>();
		}
		
		public String toString() {
			return "Node("+name+", "+position+")";
		}
		

	}
	
	/** 
	 * the edge class stores the nodes it connects, 
	 * it also keeps track of what nodes in connects
	 * in theory it could keep track of direction for some purposes
	 * but that's beyond the scope of this project
	 * it also can store it's length, but because with the graph's currect structure,
	 * it's not very convenient to find an edge between two nodes, 
	 * we instead find length with a function of two nodes
	 */
	
	public static class Edge {
		Node start;
		Node end;
		double length;
		Boolean directional = false;
		public Edge(Node start, Node end) {
			this.start = start;
			this.end = end;
		}
		
		public String toString() {
			return "Edge("+this.start+" <-> "+this.end+")";
		}
		
		
	}
	
	/**
	 * this method calculates the distance between two nodes
	 * it doesn't necessarily work well with two nodes that aren't connected
	 * but the GUI only uses it with nodes that are neighbors so it isn't a problem right now
	 * @param start
	 * @param end
	 * @return the distance between two points expressed as a double
	 */
	public Double length(Node start, Node end) {
		return Math.sqrt((Math.pow(end.position.getX()-start.position.getX(),2))+Math.pow((end.position.getY()-start.position.getY()),2));
	}
	/**
	 * this adds a new node to the graph
	 * @param nodeName
	 * @param location
	 */
	public void addNode(String nodeName, IntPoint location) {
		Node newNode = new Node(nodeName, location);
		nodes.add(newNode);
		//System.out.println(newNode);
	}
	/**
	 * this looks through every node in the graph, and if is close enough to the point clicked, it returns that node
	 * @param click
	 * @return either the node clicked or null if no node was clicked
	 */
	public Node findNode(IntPoint click) {
		if (click == null) {
			return null;
		}
		for (Node node : this.nodes) {
			if (Math.sqrt((Math.pow(click.getX()-node.position.getX(), 2)) + (Math.pow(click.getY()-node.position.getY(), 2))) < 20.0){
				return node;
			}
		}
		return null;
	}
	/**
	 * this adds an edge to the graph, stores its beginning and end node, and adds it to everywhere the graph stores edges
	 * @param origin
	 * @param destination
	 */
	public void addEdge(Node origin, Node destination) {
		System.out.println("the origin node is: " +origin);
		Edge newEdge = new Edge(origin, destination);
		System.out.println("adding an edge!");
		System.out.println("the edge is: " + newEdge);
		
		origin.edges.add(newEdge);
		//somehow the origin is null sometimes...
		
		//System.out.println(origin.edges);
		destination.edges.add(newEdge);
		origin.neighbors.add(destination);
		destination.neighbors.add(origin);
		this.edges.add(newEdge);
		}
	
	/**
	 * deletes the edge from everywhere it's stored
	 * @param edge
	 */
	public void deleteEdge(Edge edge) {
		this.edges.remove(edge);
		for (Node node : nodes) {
			if (node.edges.contains(edge)) {
				node.edges.remove(edge);
			}
		}
		
	}
	/**
	 * this deletes a node from everywhere it is stored in a graph
	 * @param node
	 */
	public void deleteNode(Node node) {
		for (int e = 0; e < edges.size(); e ++) {
			if (edges.get(e).start == node || edges.get(e).end == node) {
				this.deleteEdge(edges.get(e));
			}
		}
		for (int n = 0; n < nodes.size(); n++) {
			if (this.nodes.get(n).neighbors.contains(node)) {
				this.nodes.get(n).neighbors.remove(node);
			}
		}
		this.nodes.remove(node);
		System.out.println(this.nodes);
	}
	
	public String nodeInfo(Node node) {
		return (node.name+ "has " + node.neighbors.size()+ "neighboring edges");
	}
	
	/**
	 * this is based partly on the pseudo code from:
	 * https://brilliant.org/wiki/dijkstras-short-path-finder/
	 * and partly on the explanation from
	 * https://en.m.wikipedia.org/wiki/Dijkstra%27s_algorithm
	 * @param start - the first node you click, the start of your path
	 * @param end - the second node, the path's destination
	 * @return path, an arraylist of the nodes that the shortest path from start to end goes through
	 */
	public ArrayList<Node> Djikstra(Node start, Node end){
		ArrayList<Node> path = new ArrayList<>();
		ArrayList<Node> unvisited = new ArrayList<>();
		Node v = null;
		//first assign each node a temporary distance
		for (int i = 0; i < nodes.size(); i ++) {
			
			if (nodes.get(i) == start) {
				nodes.get(i).distance = 0;
			}
			else {
				nodes.get(i).distance = Double.POSITIVE_INFINITY;
			}
			//at the beginning, we haven't been to any of these nodes
			unvisited.add(nodes.get(i));
		}
		Double distance = unvisited.get(0).distance;
		
		while (!unvisited.isEmpty()) {
			//need to first find the unvisited node with the smallest distance
			//call this v
			int min = 0;
			for (int i = 0; i < unvisited.size(); i ++) {
				if (unvisited.get(i).distance < unvisited.get(min).distance) {
					//first time through it has to choose the start node
					min = i;
				}
			}
			v = unvisited.get(min);
			distance += v.distance;
			//then go through all of v's neighbors, reassign their distances based on 
			for (int i = 0; i < v.neighbors.size(); i ++) {
				//if the distance to the next node through v is less than that nodes existing distance, update it
				if (distance + length(v, v.neighbors.get(i)) < v.neighbors.get(i).distance) {
					//somehow we do not always set the previous so the path isn't working
					v.neighbors.get(i).distance = v.distance + length(v, v.neighbors.get(i));
					v.neighbors.get(i).previous = v;
				}
			}
			unvisited.remove(v);
			
		}
		//now in theory we should just be able to look backwards from the end node and get back to the start
		
		Node y = end;
		if (y == start || y.previous != null) {
			while (y != null) {
				path.add(0, y);
				y = y.previous;
			}
		}

		return path;
		
	}
	/**
	 * this method is very much based on the pseudo code from:https://en.wikipedia.org/wiki/Breadth-first_search
	 * @param start - the first node you click, the start of your path
	 * @param end - the second node, the path's destination
	 * @return path, an arraylist of the nodes the path from start to end goes through
	 */
	
	public ArrayList<Node> breadthFirstSearch(Node start, Node end){
		ArrayList<Node> path = new ArrayList<>();
		ArrayList<Node> queue = new ArrayList<>();
		Node first;
		start.visited = true;
		queue.add(start);
		while (! queue.isEmpty()) {
			first = queue.remove(0);
			if (first == end) {
				break;
			}
			for (Node neighbor : first.neighbors) {
				if (neighbor.visited == false) {
					neighbor.visited = true;
					neighbor.previous = first;
					queue.add(neighbor);
				}
			}
		}
		Node y = end;
		if (y == start || y.previous != null) {
			while (y != null) {
				path.add(0, y);
				y = y.previous;
			}
		}
		

		return path;
	}
	/**
	 * based on pseudo code from:https://en.wikipedia.org/wiki/Depth-first_search
	 * this is pretty similar to breadthFirst, only the array list you pull from acts like a stack, not a queue
	 * which has the affect of making you commit to each choice until you're sure it doesn't work
	 * @param start - the first node you click, the start of your path
	 * @param end - the second node, the path's destination
	 * @return path, an arraylist of the nodes the path from start to end goes through
	 */
	//
	public ArrayList<Node> depthFirstSearch(Node start, Node end){
		ArrayList<Node> stack = new ArrayList<>();
		ArrayList<Node> path = new ArrayList<>();
		stack.add(0, start);
		while (! stack.isEmpty()) {
			Node n = stack.remove(0);
			if (!n.visited) {
				n.visited = true;
				for (Node neighbor: n.neighbors) {
					neighbor.previous = n;
					stack.add(0, neighbor);
				}
			}
		}
		Node y = end;
		if (y == start || y.previous != null) {
			while (y != null) {
				path.add(0, y);
				y = y.previous;
			}
		}
		return path;
	}
	
	
	
	
	
	
	
	
	
	
	

}
