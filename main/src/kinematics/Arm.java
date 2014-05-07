package kinematics;

import java.util.LinkedList;
import java.util.List;

import javax.media.opengl.GL2;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.SingularValueDecomposition;
import org.ejml.ops.CommonOps;

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
	public DenseMatrix64F invertMatrix(DenseMatrix64F A){
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
		CommonOps.invert(temp,temp1);
		CommonOps.mult(ATrans,temp1,result);
		return result;
	}
	
	public DenseMatrix64F getJacobian(){
		//DenseMatrix64 totalRot = CommonOps.identity(0);
		//Assume there is more than one segment
		DenseMatrix64F currRot = segments.get(0).rotMatrix.copy();
		DenseMatrix64F temp = segments.get(0).getJacobian();
		DenseMatrix64F temp1 = segments.get(0).getJacobian();
		CommonOps.mult(currRot, temp1, temp);
		DenseMatrix64F totalRot = new DenseMatrix64F(3,3*numSegments);
		CommonOps.insert(temp, totalRot, 0,3*(numSegments-1));
		for(int i = 1; i < numSegments; i++){
			temp = segments.get(i).rotMatrix.copy();
			temp1 = segments.get(i).rotMatrix.copy();
			DenseMatrix64F tempcurrRot = segments.get(i).rotMatrix.copy();
			CommonOps.mult(currRot,temp,tempcurrRot);
			temp = segments.get(i).getJacobian();
			CommonOps.mult(tempcurrRot, temp1, temp);
			CommonOps.insert(temp, totalRot, 0,3*(numSegments-i-1));
		}
		
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
	
	public void solve(Point goal){
		updateJointPos();
		double epsilon = .1;
		double k = .1;
		int max_iter = 100;
		int curr = 0;
		while(segments.get(numSegments-1).end.subtract(goal).magnitude() > epsilon){
			if(curr > max_iter){
				break;
			}
			curr++;
			DenseMatrix64F rots = invertMatrix(getJacobian());
			CommonOps.scale(k, rots);
			Point diff = segments.get(numSegments-1).end.subtract(goal);
			DenseMatrix64F mat_diff = new DenseMatrix64F(3,1);
			mat_diff.set(0,0,diff.getX());
			mat_diff.set(1,0,diff.getY());
			mat_diff.set(2,0,diff.getZ());
			DenseMatrix64F result = new DenseMatrix64F(3*numSegments,1);
			CommonOps.mult(rots, mat_diff, result);
			
			//prolly make in another method
			for(int i = 0; i < numSegments; i++){
				double x = result.get(3*i,0);
				double y = result.get(3*i+1,0);
				double z = result.get(3*i+2,0);
				segments.get(i).rot = segments.get(i).rot.add(new Point(x,y,z));
			}
			updateJointPos();
		}
		/**
		 * While there is some error
		 * do stuff
		 * plz
		 * plz
		 * plz
		 */
	}

}
