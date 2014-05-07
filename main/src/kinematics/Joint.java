package kinematics;

import javax.media.opengl.GL2;

import org.ejml.data.DenseMatrix64F;

public abstract class Joint {
	
	public Point pos, end;
	public Point rot;
	// protected double theta; //angle = rot.magnitude()
	public double length;
	
	public abstract DenseMatrix64F getJacobian();
	
	public abstract void draw(GL2 gl);
	
	public abstract void updateEnd();
	
	public void setAngle(double angle){
//		theta = angle;
	}
	
	public double computeAngle(double pos){
		//TODO
		return 0.0;
	}
	
	
}
