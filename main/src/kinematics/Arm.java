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
		DenseMatrix64F result = new DenseMatrix64F(A.numCols, A.numRows);
		CommonOps.mult(A,ATrans,result);
		CommonOps.invert(result,result);
		CommonOps.mult(ATrans,result,result);
		return result;
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
		//PROBABLY DON'T NEED THIS
	}
	
	public void solve(Point p){
		/**
		 * While there is some error
		 * do stuff
		 * plz
		 * plz
		 * plz
		 */
	}

}
