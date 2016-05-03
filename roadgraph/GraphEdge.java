/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * @author rnishtal
 *
 */
public class GraphEdge {
	
	private GeographicPoint source;
	private GeographicPoint dest;
	private String roadName;
	private String roadType;
	private double length;
	
	public GraphEdge(GeographicPoint source,GeographicPoint dest,String roadName,String roadType,double len){
		this.source = source;
		this.dest = dest;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = len;
	}
	
	public GeographicPoint getSource() {
		return source;
	}
	public void setSource(GeographicPoint source) {
		this.source = source;
	}
	public GeographicPoint getDest() {
		return dest;
	}
	public void setDest(GeographicPoint dest) {
		this.dest = dest;
	}
	public String getRoadName() {
		return roadName;
	}
	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}
	public String getRoadType() {
		return roadType;
	}
	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}
	public double getLength() {
		return length;
	}
	public void setLength(double length) {
		this.length = length;
	}
		
}
