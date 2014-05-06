package kinematics;

import org.ejml.data.DenseMatrix64F;

public abstract class Joint {
	
	private double theta; //angle
	private double length;
	
	public abstract DenseMatrix64F getJacobian();
	
	public void draw(){
		//TODO
	}
	
	public void setAngle(double angle){
		theta = angle;
	}
	
	public double computeAngle(double pos){
		//TODO
		return 0.0;
	}
	
	
}
