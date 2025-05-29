package main;

import static org.lwjgl.opengl.GL15C.GL_STATIC_DRAW;

import javax.vecmath.Vector3f;

import org.joml.Matrix4f;

import com.bulletphysics.linearmath.DebugDrawModes;
import com.bulletphysics.linearmath.IDebugDraw;

import utils.VAO;

public class DebugDrawer extends IDebugDraw {
	private static final Matrix4f identity = new Matrix4f();
	private org.joml.Vector3f color = new org.joml.Vector3f();
	private Renderer renderer;
	private int debugMode;

	private VAO pointVAO;

	public DebugDrawer(Renderer renderer) {
		this.renderer = renderer;
		this.debugMode = DebugDrawModes.MAX_DEBUG_DRAW_MODE;
		pointVAO = new VAO();
		pointVAO.bind();
		pointVAO.createFloatAttribute(0, new float[] { 0, 0, 0 }, 3, 0, GL_STATIC_DRAW);
		pointVAO.unbind();
	}

	@Override
	public void drawLine(Vector3f from, Vector3f to, Vector3f color) {
		float[] line = new float[] { from.x, from.y, from.z, to.x, to.y, to.z };
		VAO lineVAO = new VAO();
		lineVAO.bind();
		lineVAO.createFloatAttribute(0, line, 3, 0, GL_STATIC_DRAW);
		lineVAO.unbind();
		this.color.set(color.x, color.y, color.z);
		renderer.renderLines(lineVAO, 2, identity, this.color, false);
		lineVAO.delete();
	}

	@Override
	public void drawContactPoint(Vector3f PointOnB, Vector3f normalOnB, float distance, int lifeTime, Vector3f color) {
		this.color.set(color.x, color.y, color.z);
		Matrix4f translation = new Matrix4f();
		translation.setTranslation(PointOnB.x, PointOnB.y, PointOnB.z);
		renderer.renderPointCloud(pointVAO, 1, translation, this.color, 5f);
	}

	@Override
	public void reportErrorWarning(String warningString) {
	}

	@Override
	public void draw3dText(Vector3f location, String textString) {
	}

	@Override
	public void setDebugMode(int debugMode) {
		this.debugMode = debugMode;
	}

	@Override
	public int getDebugMode() {
		return debugMode;
	}
}
