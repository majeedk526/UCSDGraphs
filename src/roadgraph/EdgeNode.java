package roadgraph;

import geography.GeographicPoint;

class EdgeNode implements Comparable<EdgeNode> {

	GeographicPoint from, to;
	String roadName, roadType;
	double length;
	
	public EdgeNode(GeographicPoint from ,GeographicPoint to,String roadName, String roadType, double length){
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
		
	}

	@Override
	public int compareTo(EdgeNode r) {
		// TODO Auto-generated method stub
		double currCost=length, othersCost=r.length;
		
		if(currCost==othersCost){return 0;}
		else if(currCost < othersCost) {return 1;}
		else {return -1;}
		
	}
	
}

