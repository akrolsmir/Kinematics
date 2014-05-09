package kinematics;

import javax.media.opengl.GL2;

public class Goal {

	public void draw(GL2 gl, double time) {
		Point goal = goalAt(time);

		gl.glPointSize(5);
		gl.glColor4f(0.9f, 0.9f, 0.9f, 1.0f);
		gl.glBegin(GL2.GL_POINTS);
		gl.glVertex3d(goal.getX(), goal.getY(), goal.getZ());
		gl.glEnd();
	}

	public Point goalAt(double timeInMS) {
		double angle = timeInMS * 2 * Math.PI / 5000;
		
		return new Point(Math.sin(angle), Math.cos(angle), Math.cos(angle));
	}
}
