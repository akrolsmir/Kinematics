package kinematics;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import javax.media.opengl.GL2;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

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
	
	public Point getEnd(){
		return segments.get(numSegments-1).end;
	}
	
	// Update the position of every joint that's not the first
	public void updateJointPos() {
		segments.get(0).updateEnd(null);
		for(int i = 1; i < numSegments; i++){
			Joint j = segments.get(i);
			Joint prev = segments.get(i - 1);
			j.pos = prev.end;
			j.updateEnd(prev.pos);
		}
	}
	
	/**
	 * Computes Pseudoinverse J+ = J^T(JJ^T)^-1
	 * Refer to http://math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
	 * @param A
	 * @return
	 */
	public DenseMatrix64F invertMatrix(Point goal, DenseMatrix64F A){
		/*
		SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(A.numRows, A.numCols,true,true,false);
		if(!svd.decompose(A)){
			System.err.println("Decomposition failed");
			System.exit(0);
		}
		DenseMatrix64F U = svd.getU(null,false);
		DenseMatrix64F W = svd.getW(null);
		DenseMatrix64F V = svd.getV(null,false);
		*/
		
		//holy shit this library
		DenseMatrix64F ATrans = new DenseMatrix64F(A.numCols, A.numRows);
		CommonOps.transpose(A,ATrans);
		DenseMatrix64F temp = new DenseMatrix64F(A.numRows, A.numRows);
		DenseMatrix64F temp1 = new DenseMatrix64F(A.numRows, A.numRows);
		DenseMatrix64F result = new DenseMatrix64F(A.numCols, A.numRows);
		CommonOps.mult(A,ATrans,temp);
		//System.out.println(CommonOps.det(temp));
		if(Math.abs(CommonOps.det(temp)) < 0.0000001 ){
			System.out.println(CommonOps.det(temp));
			System.out.println("hi");
			for(Joint j : segments){
				j.rot = j.rot.Perturb(.001);
				j.makeRotMatrix();
			}
			updateJointPos();
			return invertMatrix(goal, getJacobian(goal));
		}
		CommonOps.invert(temp,temp1);
		CommonOps.mult(ATrans,temp1,result);
		return result;
	}
	
	public DenseMatrix64F getJacobian(Point goal){
		//DenseMatrix64 totalRot = CommonOps.identity(0);
		//Assume there is more than one segment
		double len = 0.0;
		for(int i = 0; i < numSegments; i++){
			len += segments.get(i).length;
		}
		DenseMatrix64F currRot = segments.get(0).rotMatrix.copy();
		DenseMatrix64F temp = segments.get(0).getJacobian(goal).copy();
		DenseMatrix64F temp1 = segments.get(0).getJacobian(goal).copy();
		CommonOps.mult(currRot, temp1, temp);
		DenseMatrix64F totalRot = new DenseMatrix64F(3,3*numSegments);
		CommonOps.insert(temp, totalRot, 0, 0);
		for(int i = 1; i < numSegments; i++){
			len -= segments.get(i-1).length;
			//temp = segments.get(i).rotMatrix.copy();
			temp1 = segments.get(i).rotMatrix.copy();
			DenseMatrix64F tempcurrRot = segments.get(i).rotMatrix.copy();
			//CommonOps.mult(currRot,temp,tempcurrRot);
			temp = segments.get(i).getJacobian(goal).copy();
			CommonOps.transpose(tempcurrRot);
			DenseMatrix64F temp2 = temp.copy();
			CommonOps.mult(tempcurrRot, temp, temp2);
			CommonOps.insert(temp2, totalRot, 0,3*i);
			//CommonOps.insert(temp, totalRot, 0,3*i);
		}
		//System.out.println(totalRot);
		
		return totalRot;
	}
	
	/**
	 * Does a step towards the goal
	 * @param perturb
	 */
	public void doStep(double perturb){
		//TODO
		//PROBABLY DON'T NEED THIS
	}
	
	public double getLength(){
		double len = 0.0;
		for(Joint j : segments){
			len += j.length;
		}
		return len;
	}
	
	public void solve(Point goal, GL2 gl, double ep){
		updateJointPos();
		//double epsilon = .2;
		double k = 1.0;
		int max_iter = 1000;
		int curr = 0;
		int num = 1;
		if(goal.magnitude() > getLength()){
			solve(goal.normalize().multiply(getLength()*.98), gl, ep);
			return;
		}
		Point orig = segments.get(numSegments-1).end.add(Point.ZERO);
		ArrayList<Point>orig_rots = new ArrayList<Point>();
		for(Joint j : segments){
			orig_rots.add(j.rot);
		}
		while(getEnd().subtract(goal).magnitude() > ep){
			//System.out.println(segments.get(numSegments-1).end.subtract(goal).magnitude());
			
			if(curr > max_iter){
				if(num > 5){
					System.out.println("hello");
					//give up
					for(int i = 0; i < numSegments; i++){
						segments.get(i).rot = orig_rots.get(i);
						segments.get(i).makeRotMatrix();
					}
					updateJointPos();
					solve(goal.multiply(.5).add(orig.multiply(.5)), gl, ep);
					return;
				}
				num++;
				//draw(gl);
				for(Joint j : segments){
					j.rot = j.rot.Perturb(.01);
					j.makeRotMatrix();
				}
				updateJointPos();
				curr = 0;
			}
			curr++;
			DenseMatrix64F rots = invertMatrix(goal, getJacobian(goal));
			//System.out.println(rots);
			CommonOps.scale(k, rots);
			Point diff = goal.subtract(getEnd());
			DenseMatrix64F mat_diff = new DenseMatrix64F(3,1);
			mat_diff.set(0,0,diff.getX());
			mat_diff.set(1,0,diff.getY());
			mat_diff.set(2,0,diff.getZ());
			DenseMatrix64F result = new DenseMatrix64F(3*numSegments,1);
			CommonOps.mult(rots, mat_diff, result);
			if(NormOps.elementP(result,2) > 100){
				System.out.println("Difference too great");
				solve(orig, gl, ep);
				return;
			}
			//System.out.println(getJacobian());
			
			//prolly make in another method
			for(int i = 0; i < numSegments; i++){
				double x = result.get(3*i,0);
				double y = result.get(3*i+1,0);
				double z = result.get(3*i+2,0);
				segments.get(i).rot = segments.get(i).rot.add(new Point(x,y,z));
			}
			updateJointPos();
		}
		draw(gl);
	}

}
