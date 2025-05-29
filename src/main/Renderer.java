package main;

import static org.lwjgl.opengl.GL46C.*;

import java.util.List;

import org.joml.Matrix4f;
import org.joml.Matrix4fc;
import org.joml.Vector3f;
import org.joml.Vector4f;

import utils.Shader;
import utils.VAO;

public class Renderer {
	public static final Vector3f BLACK = new Vector3f(0, 0, 0), //
			WHITE = new Vector3f(1, 1, 1), //
			GRAY = new Vector3f(0.5f, 0.5f, 0.5f), //
			RED = new Vector3f(1, 0, 0), //
			GREEN = new Vector3f(0, 1, 0), //
			BLUE = new Vector3f(0, 0, 1); //

	private Shader shader;

	public Renderer() {
		shader = new Shader("res/shaders/simple.vs", "res/shaders/simple.fs");
		shader.bindVertexAttribute(0, "position");
		shader.finishInit();
		shader.init_uniforms(List.of("projectionMatrix", "viewMatrix", "modelMatrix", "color"));
	}

	public void start(Matrix4fc projectionMatrix, Matrix4fc viewMatrix) {
		shader.start();
		shader.loadMat4("projectionMatrix", projectionMatrix);
		shader.loadMat4("viewMatrix", viewMatrix);
	}

	public void renderObject(VAO object, Matrix4f modelMatrix, Vector3f color, boolean edges) {
		object.bind();
		object.bindAttribute(0);
		// Faces
		shader.loadVec3("color", color);
		shader.loadMat4("modelMatrix", modelMatrix);
		glDrawElements(GL_TRIANGLES, object.getIndexCount(), GL_UNSIGNED_INT, 0L);

		if (edges) {
			// Edges
			glLineWidth(1.0f);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);// Line mode
			glPolygonOffset(0, -1.0f);// Draw the edges slightly above the faces
			shader.loadVec3("color", BLACK);
			glDrawElements(GL_TRIANGLES, object.getIndexCount(), GL_UNSIGNED_INT, 0L);
			glPolygonOffset(0, 0.0f);// Back to normal
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);// Back to filled polygons
		}

//		//Vertices
//		glPointSize(4.0f);
//		glPolygonOffset(0, -10.0f);//Even more above
//		shader.loadVec4("color", new Vector4f(0, 0, 0, 1));
//		glDrawElements(GL_POINTS, vao.getIndexCount(), GL_UNSIGNED_INT, 0L);
//		glPolygonOffset(0, 0.0f);//Back to normal

		object.unbindAttribute(0);
		object.unbind();
	}

	public void renderPointCloud(VAO cloud, int count, Matrix4f modelMatrix, Vector3f color) {
		renderPointCloud(cloud, count, modelMatrix, color, 4f);
	}
	
	public void renderPointCloud(VAO cloud, int count, Matrix4f modelMatrix, Vector3f color, float pointSize) {
		cloud.bind();
		cloud.bindAttribute(0);
		glPointSize(pointSize);
		shader.loadMat4("modelMatrix", modelMatrix);
		shader.loadVec3("color", color);
		glDrawArrays(GL_POINTS, 0, count);
		cloud.unbindAttribute(0);
		cloud.unbind();
	}

	public void renderLines(VAO lines, int count, Matrix4f modelMatrix, Vector3f color, boolean strip) {
		lines.bind();
		lines.bindAttribute(0);
		glLineWidth(4.0f);
		shader.loadMat4("modelMatrix", modelMatrix);
		shader.loadVec3("color", color);
		glDrawArrays(strip ? GL_LINE_STRIP : GL_LINES, 0, count);
		lines.unbindAttribute(0);
		lines.unbind();
	}

	public void stop() {
		shader.stop();
	}

	public void destroy() {
		shader.delete();
	}
}
