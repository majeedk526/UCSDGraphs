package roadgraph;

import java.util.List;

import geography.GeographicPoint;

class MapNode {

	GeographicPoint gpNode;
	List<EdgeNode> neighbours;
	
	
	public MapNode(GeographicPoint gp, GeographicPoint start){
		this.gpNode = gp;
		
	}
	

}
