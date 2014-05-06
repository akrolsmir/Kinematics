package kinematics;

import java.util.LinkedList;
import java.util.List;

import javax.media.opengl.GL2;

import org.ejml.data.DenseMatrix64F;

public class Arm {
	
	protected List<Joint> segments;
	private int numSegments;
	
	public Arm(List<Joint> segs){
		//copy into segments
		segments = new LinkedList<Joint>();
		segments.addAll(segs);
		numSegments = segments.size();
	}
	
	public void draw(GL2 gl){
		for(Joint j : segments){
			j.draw(gl);
		}
	}
	
	public DenseMatrix64F getJacobian(){
		//TODO
		return new DenseMatrix64F(numSegments,numSegments);
	}
	
	/**
	 * Does a step towards the goal
	 * @param perturb
	 */
	public void doStep(double perturb){
		//TODO
	}
	
	public void solve(Point p){
		//TODO
	}

}
