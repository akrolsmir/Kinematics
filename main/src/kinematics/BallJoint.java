package kinematics;

import org.ejml.data.DenseMatrix64F;

public class BallJoint {

	public DenseMatrix64F getJacobian(){
		return new DenseMatrix64F(1,1);
	}
}
