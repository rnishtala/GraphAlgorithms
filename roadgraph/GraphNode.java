/**
 * 
 */
package roadgraph;

import java.util.*;

import geography.GeographicPoint;

/**
 * @author rnishtal
 *
 */
public class GraphNode {
	private GeographicPoint location;
	private List<GraphEdge> fromEdges;
	private Integer priority;
	
	public GraphNode(GeographicPoint point){
		location = point;
	}
	
	public GeographicPoint getLocation() {
		return location;
	}
	public void setLocation(GeographicPoint location) {
		this.location = location;
	}
	public Integer getPriority() {
		return priority;
	}

	public void setPriority(Integer priority) {
		this.priority = priority;
	}

	public List<GraphEdge> getFromEdges() {
		return fromEdges;
	}

	public void setFromEdges(List<GraphEdge> fromEdges) {
		this.fromEdges = fromEdges;
	}
	
	public boolean matchNode(GeographicPoint location){
		if(this.location == location){
			return true;
		}
		return false;
	}
		
}
