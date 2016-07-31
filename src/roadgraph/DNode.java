package roadgraph;

class DNode implements Comparable<DNode>{

	Route r;
	DNode parent;
	double edgeCost;
	
	public DNode(Route r, double edgeCost, DNode parent){
		this.r = r;
		this.edgeCost = edgeCost;
		this.parent = parent;
	}

	@Override
	public int compareTo(DNode d) {
		// TODO Auto-generated method stub
		
		double currCost=edgeCost, othersCost=d.edgeCost;
		
		if(currCost==othersCost){return 0;}
		else if(currCost < othersCost) {return -1;}
		else {return 1;}
	}
	
	@Override
	public String toString() {
		// TODO Auto-generated method stub
		String s = null;
		if(parent!=null){
			s = "curr : " + r.to.x + " " + r.to.y + " edgeCost : " + edgeCost + " parent : " + parent.r.to.x 
					+ " " + parent.r.to.y;
		}else {
			s = "curr : " + r.to.x + " " + r.to.y + " edgeCost : " + edgeCost + " parent : null";
		}
		
		return s;
	}
	
}
