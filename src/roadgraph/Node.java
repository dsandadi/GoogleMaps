package roadgraph;


import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class Node implements Comparable<Node> {
	private GeographicPoint location;
	private ArrayList<Edge> edges;
	private double currentDistance=Double.POSITIVE_INFINITY;//distance from start to current
	private double prediction = Double.POSITIVE_INFINITY;//distance from current to goal.

	public Node(GeographicPoint location) {
		super();
		this.location = location;
		edges = new ArrayList<>();
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public void setLocation(GeographicPoint location) {
		this.location = location;
	}

	public ArrayList<Edge> getEdges() {
		return edges;
	}

	public void setEdges(ArrayList<Edge> edges) {
		this.edges = edges;
	}

	public double getCurrentDistance() {
		return currentDistance;
	}

	public void setCurrentDistance(double currentDistance) {
		this.currentDistance = currentDistance;
	}

	public double findDistance(Node goal)
	{
		GeographicPoint a = this.getLocation();
		return a.distance(goal.getLocation());
		
	}
	private List<Node> getNeigbors()
	{
		List<Node> neighbors = new ArrayList<Node>();
		for(Edge e:edges)
		{
			Node end = e.getEnd();
			if(end != this)
			neighbors.add(end);
		}
		return neighbors;
	}
	@Override
	public int compareTo(Node o) {
		// TODO Auto-generated method stub
		double value = this.currentDistance-o.currentDistance;
		if(value>0)
			return 1;
		else if(value<0) return -1;
		else return 0;
		
	}

	public double getPrediction() {
		return prediction;
	}

	public void setPrediction(double prediction) {
		this.prediction = prediction;
	}
	//public void initializePredictions(Node goalNode)
	//{
		
	//}
}