package roadgraph;

import geography.GeographicPoint;

class Route implements Comparable<Route>{
	GeographicPoint from, to;
	String roadName, roadType;
	double length;
	
	public Route(GeographicPoint from ,GeographicPoint to,String roadName, String roadType, double length){
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
		//this.edgeCost = edgeCost;
		
	}
	
	

	@Override
	public int compareTo(Route r) {
		// TODO Auto-generated method stub
		double currCost=length, othersCost=r.length;
		
		if(currCost==othersCost){return 0;}
		else if(currCost < othersCost) {return -1;}
		else {return 1;}
		
	}
	
	
	

}
