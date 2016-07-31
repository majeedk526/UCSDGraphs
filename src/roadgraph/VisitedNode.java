package roadgraph;

import geography.GeographicPoint;

class VisitedNode {
	
	VisitedNode parent;
	GeographicPoint gpNode;
	DNode dnode;
	
	public VisitedNode(GeographicPoint gpNode, VisitedNode p){
		this.gpNode = gpNode;
		this.parent = p;
		
	}
	
	public VisitedNode(DNode dn, VisitedNode p){
		this.parent = p;
		this.dnode = dn;
	}
}
