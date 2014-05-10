package kinematics;

import javax.media.opengl.GL2;

public class Goal {

	public static void draw(Arm arm, GL2 gl, double time) {
		Point goal = goalAt(time);
		
		arm.updateJointPos();
		arm.solve(goal, gl, .2);

		gl.glPointSize(5);
		gl.glColor4f(0.9f, 0.9f, 0.9f, 1.0f);
		gl.glBegin(GL2.GL_POINTS);
		gl.glColor3d(1.0, 0.0, 0.0);
		gl.glVertex3d(goal.getX(), goal.getY(), goal.getZ());
		gl.glEnd();
	}

	public static Point goalAt(double timeInMS) {
		double angle = timeInMS * 2 * Math.PI / 10000;
		
		return new Point(4 * Math.sin(0.5 * angle), 2 * Math.sin(angle), 0);
	}
}
