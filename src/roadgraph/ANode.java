package roadgraph;

import geography.GeographicPoint;

class ANode implements Comparable<ANode> {
	
	Route r;
	ANode parent;
	double G, H;
	
	public ANode(Route r, ANode parent, double G, double H){
		this.r = r;
		this.G = G;
		this.H = H;
		this.parent = parent;
	}

	@Override
	public int compareTo(ANode a) {
		// TODO Auto-generated method stub
	
		double distCurr = G+H, distOther = a.G + a.H;
		
		if(distCurr==distOther){return 0;}
		else if(distCurr < distOther) {return -1;}
		else {return 1;}
	}
	
	@Override
	public String toString() {
		// TODO Auto-generated method stub
		String s = null;
		if(parent!=null){
			s = "curr : " + r.to.x + " " + r.to.y + " G+H : " + (G+H) + " parent : " + parent.r.to.x 
					+ " " + parent.r.to.y;
		}else {
			s = "curr : " + r.to.x + " " + r.to.y + " G+H : " + (G+H) + " parent : null";
		}
		
		return s;
	}

}
