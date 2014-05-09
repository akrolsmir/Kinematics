package kinematics;

import java.awt.*;
import java.awt.event.*;
import java.util.Arrays;

import javax.media.opengl.*;
import javax.media.opengl.awt.GLCanvas;
import javax.media.opengl.fixedfunc.*;

import com.jogamp.opengl.util.Animator;
 
/**
 * Self-contained example (within a single class only to keep it simple) 
 * displaying a rotating quad
 */
public class Renderer implements GLEventListener {
	
	static double angle = 0.0;
	
	Joint ballJoint = new BallJoint(1, new Point(0, 0, 0), new Point(1, 1, 1));
	static Arm arm = new Arm(Arrays.asList(new Joint[]{
			new BallJoint(1, new Point(0, 0, 0), new Point(0, 0, 0)),
			new BallJoint(3, new Point(0, 1, 0), new Point(0, 0, 1.57)),
			new BallJoint(2, new Point(1, 0, 0), new Point(3, 0, 0)),
			new BallJoint(1, new Point(0, 1, 0), new Point(0.5, 0.2, 1.0)),
	}));
	
	static Point goal;
	double startTime = System.currentTimeMillis();
 
	@Override
	public void display(GLAutoDrawable gLDrawable) {
		final GL2 gl = gLDrawable.getGL().getGL2();
		gl.glClear(GL.GL_COLOR_BUFFER_BIT);
		gl.glClear(GL.GL_DEPTH_BUFFER_BIT);
		gl.glLoadIdentity();
		gl.glTranslatef(0.0f, 0.0f, -5.0f);
 
		Goal.draw(arm, gl, System.currentTimeMillis() - startTime);
	}
 
	@Override
	public void init(GLAutoDrawable glDrawable) {
		GL2 gl = glDrawable.getGL().getGL2();
		gl.glShadeModel(GLLightingFunc.GL_SMOOTH);
		gl.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		gl.glClearDepth(1.0f);
		gl.glEnable(GL.GL_DEPTH_TEST);
		gl.glDepthFunc(GL.GL_LEQUAL);
		gl.glHint(GL2ES1.GL_PERSPECTIVE_CORRECTION_HINT, GL.GL_NICEST);
	}
 
	@Override
	public void reshape(GLAutoDrawable gLDrawable, int x, int y, int width, int height) {
		GL2 gl = gLDrawable.getGL().getGL2();
		final float aspect = (float) width / (float) height;
		gl.glMatrixMode(GLMatrixFunc.GL_PROJECTION);
		gl.glLoadIdentity();
		final float fh = 0.5f;
		final float fw = fh * aspect;
		gl.glFrustumf(-fw, fw, -fh, fh, 1.0f, 1000.0f);
		gl.glMatrixMode(GLMatrixFunc.GL_MODELVIEW);
		gl.glLoadIdentity();
	}
 
	@Override
	public void dispose(GLAutoDrawable gLDrawable) {
	}
 
	public static void main(String[] args) {
		arm.updateJointPos();
		goal = arm.getEnd().add(Point.ZERO);
		final GLCanvas canvas = new GLCanvas();
		final Frame frame = new Frame("Jogl Quad drawing");
		final Animator animator = new Animator(canvas);
		canvas.addGLEventListener(new Renderer());
		frame.add(canvas);
		frame.setSize(640, 480);
		frame.setResizable(false);
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				animator.stop();
				frame.dispose();
				System.exit(0);
			}
		});
		canvas.addKeyListener(new KeyAdapter() {
			@Override
			public void keyPressed(KeyEvent e) {
				double ep = .1;
				switch (e.getKeyCode()) {
				case KeyEvent.VK_UP:
					goal = new Point(goal.getX(), goal.getY()+ep,goal.getZ());
					break;
				case KeyEvent.VK_DOWN:
					goal = new Point(goal.getX(), goal.getY()-ep,goal.getZ());
					break;
				case KeyEvent.VK_LEFT:
					goal = new Point(goal.getX()-ep, goal.getY(),goal.getZ());
					break;
				case KeyEvent.VK_RIGHT:
					goal = new Point(goal.getX()+ep, goal.getY(),goal.getZ());
					break;
				case KeyEvent.VK_EQUALS:
					goal = new Point(goal.getX(), goal.getY(),goal.getZ()+ep);
					break;
				case KeyEvent.VK_MINUS:
					goal = new Point(goal.getX(), goal.getY(),goal.getZ()-ep);
					break;
				}
			}
		});

		frame.setVisible(true);
		animator.start();
		canvas.requestFocus();
	}
}