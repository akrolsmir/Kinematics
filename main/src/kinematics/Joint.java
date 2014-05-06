package kinematics;

import javax.media.opengl.GL2;

import org.ejml.data.DenseMatrix64F;

public abstract class Joint {
	
	protected Point pos;
	protected Point rot;
	// protected double theta; //angle = rot.magnitude()
	protected double length;
	
	public abstract DenseMatrix64F getJacobian();
	
	public abstract void draw(GL2 gl);
	
	public void setAngle(double angle){
//		theta = angle;
	}
	
	public double computeAngle(double pos){
		//TODO
		return 0.0;
	}
	
	
}
