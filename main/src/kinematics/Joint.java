package kinematics;

import javax.media.opengl.GL2;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public abstract class Joint {
	
	public Point pos, end;
	public Point rot;
	// protected double theta; //angle = rot.magnitude()
	public DenseMatrix64F rotMatrix;
	public double length;
	
	public abstract DenseMatrix64F getJacobian();
	
	public abstract void draw(GL2 gl);
	
	public abstract void updateEnd(Point prevPos);
	
	public void setAngle(double angle){
//		theta = angle;
	}
	
	public DenseMatrix64F rotJacobian(DenseMatrix64F rotation){
		DenseMatrix64F result = new DenseMatrix64F(rotMatrix.numCols, rotMatrix.numRows);
		CommonOps.mult(rotation, rotMatrix, result);
		CommonOps.mult(result, getJacobian(), result);
		return result;
	}
	
	public double computeAngle(double pos){
		//TODO
		return 0.0;
	}
	
	
}
