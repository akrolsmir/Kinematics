package kinematics;

import javax.media.opengl.GL2;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class BallJoint extends Joint {
	
	public BallJoint (double length, Point pos, Point rot){
		this.length = length;
		this.pos = pos;
		this.rot = rot;
		this.rotMatrix = CommonOps.identity(3);
		Point rotNorm = rot.normalize();
		//CommonOps.add
		double[][] rot_array = new double[][]{{0,-rotNorm.getZ(),rotNorm.getY()},
				{rotNorm.getZ(),0,-rotNorm.getX()},
				{-rotNorm.getY(),rotNorm.getX(),0}};
		DenseMatrix64F temp = new DenseMatrix64F(rot_array);
		DenseMatrix64F temp1 = new DenseMatrix64F(rot_array);
		DenseMatrix64F temp2 = new DenseMatrix64F(rot_array);
		DenseMatrix64F temp3 = new DenseMatrix64F(rot_array);
		CommonOps.scale(Math.sin(rot.magnitude()), temp1);
		CommonOps.add(this.rotMatrix, temp1, this.rotMatrix);
		CommonOps.mult(temp, temp2, temp3);
		CommonOps.scale(1-Math.cos(rot.magnitude()), temp3);
		CommonOps.add(this.rotMatrix, temp3, this.rotMatrix);
		
	}

	@Override
	public DenseMatrix64F getJacobian(){
		double[][] pos_array = new double[][]{{0,pos.getZ(),-pos.getY()},
				{-pos.getZ(),0,pos.getX()},
				{pos.getY(),-pos.getX(),0}};
		return new DenseMatrix64F(pos_array);
	}
	
	@Override
	// Calculate endpoint with Rodriguez formula
	public void updateEnd(Point prevPos) {
		Point x = prevPos == null ? new Point(length, 0, 0) : pos.subtract(prevPos);
		double theta = rot.magnitude();
		if (theta == 0.0) {
			end = x;
		} else {
			Point norm =  rot.normalize();
			end = norm.multiply((norm.dotProduct(x)))
					.add(norm.crossProduct(x).multiply(Math.sin(theta)))
					.subtract(norm.crossProduct(norm.crossProduct(x)).multiply(Math.cos(theta)));
		}
		end = pos.add(end);
	}

	@Override
	public void draw(GL2 gl) {
		// Draw a line between pos and end
		gl.glBegin(GL2.GL_LINES);
		gl.glVertex3d(pos.getX(), pos.getY(), pos.getZ());
		gl.glVertex3d(end.getX(), end.getY(), end.getZ());
		gl.glEnd();
	}
}
