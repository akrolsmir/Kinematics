package kinematics;

import javax.media.opengl.GL2;

import org.ejml.data.DenseMatrix64F;

public class BallJoint extends Joint {
	
	public BallJoint (double length, Point pos, Point rot){
		this.length = length;
		this.pos = pos;
		this.rot = rot;
	}

	@Override
	public DenseMatrix64F getJacobian(){
		return new DenseMatrix64F(1,1);
	}

	@Override
	public void draw(GL2 gl) {
		// Calculate endpoint with Rodriguez formula
		Point x = new Point(length, 0, 0);
		double theta = rot.magnitude();
		Point norm =  rot.normalize();
		Point end = norm.multiply((norm.dotProduct(x)))
				.add(norm.crossProduct(x).multiply(Math.sin(theta)))
				.subtract(norm.crossProduct(norm.crossProduct(x)).multiply(Math.cos(theta)));
		end = pos.add(end);
		
		// Draw a line between pos and end
		gl.glBegin(GL2.GL_LINES);
		gl.glVertex3d(pos.getX(), pos.getY(), pos.getZ());
		gl.glVertex3d(end.getX(), end.getY(), end.getZ());
		gl.glEnd();
	}
}
