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
