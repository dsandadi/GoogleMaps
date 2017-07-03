package roadgraph;

public class Edge {

	private Node start;
	private Node end;
	private String name;
	private double length;
	private String roadType;

	public Edge(Node start, Node end, String name, double length, String roadType) {
		super();
		this.start = start;
		this.end = end;
		this.name = name;
		this.length = length;
		this.roadType = roadType;
	}

	public Node getStart() {
		return start;
	}

	public void setStart(Node start) {
		this.start = start;
	}

	public Node getEnd() {
		return end;
	}

	public void setEnd(Node end) {
		this.end = end;
	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}

	public double getLength() {
		return length;
	}

	public void setLength(double length) {
		this.length = length;
	}

	public String getRoadType() {
		return roadType;
	}

	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}

}
