package main;

import static main.Renderer.*;
import static main.Constants.*;
import static org.lwjgl.glfw.GLFW.GLFW_CONTEXT_VERSION_MAJOR;
import static org.lwjgl.glfw.GLFW.GLFW_CONTEXT_VERSION_MINOR;
import static org.lwjgl.glfw.GLFW.GLFW_DECORATED;
import static org.lwjgl.glfw.GLFW.GLFW_FALSE;
import static org.lwjgl.glfw.GLFW.GLFW_OPENGL_CORE_PROFILE;
import static org.lwjgl.glfw.GLFW.GLFW_OPENGL_FORWARD_COMPAT;
import static org.lwjgl.glfw.GLFW.GLFW_OPENGL_PROFILE;
import static org.lwjgl.glfw.GLFW.GLFW_RESIZABLE;
import static org.lwjgl.glfw.GLFW.GLFW_SAMPLES;
import static org.lwjgl.glfw.GLFW.GLFW_SCALE_TO_MONITOR;
import static org.lwjgl.glfw.GLFW.GLFW_TRUE;
import static org.lwjgl.glfw.GLFW.GLFW_VISIBLE;
import static org.lwjgl.glfw.GLFW.glfwCreateWindow;
import static org.lwjgl.glfw.GLFW.glfwDefaultWindowHints;
import static org.lwjgl.glfw.GLFW.glfwGetFramebufferSize;
import static org.lwjgl.glfw.GLFW.glfwGetTime;
import static org.lwjgl.glfw.GLFW.glfwInit;
import static org.lwjgl.glfw.GLFW.glfwMakeContextCurrent;
import static org.lwjgl.glfw.GLFW.glfwMaximizeWindow;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.glfw.GLFW.glfwSwapInterval;
import static org.lwjgl.glfw.GLFW.glfwTerminate;
import static org.lwjgl.glfw.GLFW.glfwWindowHint;
import static org.lwjgl.opengl.GL11C.GL_COLOR_BUFFER_BIT;
import static org.lwjgl.opengl.GL11C.GL_DEPTH_BUFFER_BIT;
import static org.lwjgl.opengl.GL11C.GL_DEPTH_TEST;
import static org.lwjgl.opengl.GL11C.GL_RENDERER;
import static org.lwjgl.opengl.GL11C.GL_TRIANGLES;
import static org.lwjgl.opengl.GL11C.GL_UNSIGNED_INT;
import static org.lwjgl.opengl.GL11C.glClear;
import static org.lwjgl.opengl.GL11C.glClearColor;
import static org.lwjgl.opengl.GL11C.glDrawElements;
import static org.lwjgl.opengl.GL11C.glEnable;
import static org.lwjgl.opengl.GL11C.glGetInteger;
import static org.lwjgl.opengl.GL11C.glGetString;
import static org.lwjgl.opengl.GL11C.glViewport;
import static org.lwjgl.opengl.GL30C.GL_MAJOR_VERSION;
import static org.lwjgl.opengl.GL30C.GL_MINOR_VERSION;
import static org.lwjgl.system.MemoryUtil.NULL;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.List;

import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import org.joml.Quaternionf;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryStack;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CylinderShape;
import com.bulletphysics.collision.shapes.CylinderShapeX;
import com.bulletphysics.collision.shapes.CylinderShapeZ;
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.collision.shapes.TriangleIndexVertexArray;
import com.bulletphysics.collision.shapes.TriangleMeshShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;

import imgui.ImGui;
import imgui.app.Application;
import imgui.app.Configuration;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.ImPlotContext;
import imgui.flag.ImGuiSliderFlags;
import utils.Camera;
import utils.ModelLoader;
import utils.Shader;
import utils.Texture;
import utils.VAO;
import vehicle.Car;

public class DiffDriveMain extends Application {
	public static void main(String args[]) {
		launch(new DiffDriveMain());
	}

	private int frameWidth;
	private int frameHeight;

	private DynamicsWorld dynamicsWorld;
	private int[] substeps = new int[] { 20 };
	private Car vehicle;

	private Camera camera;
	private Renderer renderer;
	private boolean renderDebug = false;
	
	private VAO planeVAO;
	private Shader planeShader;
	private Texture floorTexture;

	private Vector3f bumpPos = new Vector3f(1.5f, -1.8f, 10f);
	private VAO bumpVAO;
	private float bumpX = 2f;
	private float bumpY = 2f;
	private float bumpZ = 2f;

	@Override
	protected void configure(Configuration config) {
		config.setTitle("Simulation physique");
	}

	@Override
	protected void initWindow(Configuration config) {
		GLFWErrorCallback.createPrint(System.err).set();

		if (!glfwInit())
			throw new IllegalStateException("Failed to initialize GLFW");

		glfwDefaultWindowHints();
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
		glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
		glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
		glfwWindowHint(GLFW_DECORATED, GLFW_TRUE);
		glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
		glfwWindowHint(GLFW_SAMPLES, 8);

		handle = glfwCreateWindow(640, 480, "Simulation physique", NULL, NULL);

		if (handle == NULL) {
			glfwTerminate();
			throw new NullPointerException("Window pointer is NULL");
		}

		glfwMakeContextCurrent(handle);
		GL.createCapabilities();

		glfwShowWindow(handle);
		glfwSwapInterval(1);

		if (config.isFullScreen()) {
			glfwMaximizeWindow(handle);
		} else {
			glfwShowWindow(handle);
		}

		glClearColor(colorBg.getRed(), colorBg.getGreen(), colorBg.getBlue(), colorBg.getAlpha());
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	@Override
	protected void preRun() {
		initImPlot();
		initResources();
		initPhysics();
	}

	private void initImPlot() {
		ImPlot.createContext();
	}

	private void initPhysics() {
		// créer le dynamics world
		BroadphaseInterface broadPhase = new DbvtBroadphase();
		CollisionConfiguration collisionConfig = new DefaultCollisionConfiguration();
		CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfig);
		ConstraintSolver solver = new SequentialImpulseConstraintSolver();
		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadPhase, solver, collisionConfig);
		dynamicsWorld.setGravity(GRAVITY);
		dynamicsWorld.setDebugDrawer(new DebugDrawer(renderer));
		// créer le sol, un plan horizontal infini
		{
			CollisionShape groundShape = new StaticPlaneShape(Y_AXIS, 0.001f);
			MotionState groundMotionState = new DefaultMotionState(new Transform(//
					new Matrix4f(//
							new Quat4f(0, 0, 0, 1), //
							ZERO, //
							1.0f)));//
			RigidBodyConstructionInfo groundRBInfo = new RigidBodyConstructionInfo( //
					0, // pas de masse
					groundMotionState, //
					groundShape, //
					ZERO); // pas de moment d'inertie

			groundRBInfo.restitution = 0;
			groundRBInfo.friction = 1f;
			RigidBody groundRB = new RigidBody(groundRBInfo);
			dynamicsWorld.addRigidBody(groundRB);
		}

		// un obstacle
		{
			CollisionShape shape = new CylinderShapeX(new Vector3f(bumpX, bumpY, bumpZ));
			MotionState state0 = new DefaultMotionState(new Transform(//
					new Matrix4f(//
							new Quat4f(0, 0, 0, 1), //
							bumpPos, //
							1.0f)));//
			RigidBodyConstructionInfo info = new RigidBodyConstructionInfo( //
					0, // pas de masse
					state0, //
					shape, //
					ZERO); // pas de moment d'inertie

			info.restitution = 0;
			info.friction = 1f;
			RigidBody bump = new RigidBody(info);
			bump.setCollisionFlags(CollisionFlags.STATIC_OBJECT);
			dynamicsWorld.addRigidBody(bump);
		}

		vehicle = new Car(handle, dynamicsWorld);
	}

	private void initResources() {
		camera = new Camera(handle);
		renderer = new Renderer();
		planeVAO = ModelLoader.load("res/models/plane.obj", 0, true);
		bumpVAO = ModelLoader.load("res/models/unit_cylinder.obj", 0, true);
		planeShader = new Shader("res/shaders/floor.vs", "res/shaders/floor.fs");
		planeShader.bindVertexAttribute(0, "position");
		planeShader.finishInit();
		planeShader.init_uniforms(List.of("projectionMatrix", "viewMatrix", "modelMatrix"));

		try {
			floorTexture = new Texture("res/textures/floor.png");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void process() {
		update();

		layoutGUI();
		MemoryStack stack = MemoryStack.stackPush();
		IntBuffer width = stack.mallocInt(1);
		IntBuffer height = stack.mallocInt(1);
		glfwGetFramebufferSize(handle, width, height);
		frameWidth = width.get(0);
		frameHeight = height.get(0);
		stack.pop();

		glViewport(0, 0, frameWidth, frameHeight);
		render();
	}

	private void update() {
		float timeStep = FRAME_TIME_STEP / substeps[0];
		vehicle.update(timeStep);
		for (int i = 0; i < substeps[0]; i++) {
			dynamicsWorld.stepSimulation(timeStep, 0);
		}
	}

	private void render() {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		camera.setVehiclePos(vehicle.getCarPosition());
		camera.setVehicleDir(vehicle.getCarDirection());

		camera.updateView(frameWidth, frameHeight, !ImGui.isWindowFocused());

		glEnable(GL_DEPTH_TEST);

		planeShader.start();
		org.joml.Matrix4f modelMatrix = new org.joml.Matrix4f();
		modelMatrix.scale(100);
		planeVAO.bind();
		planeVAO.bindAttribute(0);

		planeShader.start();
		planeShader.loadMat4("projectionMatrix", camera.getProjectionMatrix());
		planeShader.loadMat4("viewMatrix", camera.getViewMatrix());
		planeShader.loadMat4("modelMatrix", modelMatrix);
		floorTexture.bindAsTexture(0);

		glDrawElements(GL_TRIANGLES, planeVAO.getIndexCount(), GL_UNSIGNED_INT, 0L);
		floorTexture.unbindAsTexture(0);

		planeVAO.unbindAttribute(0);
		planeVAO.unbind();

		planeShader.stop();

		renderer.start(camera.getProjectionMatrix(), camera.getViewMatrix());

		modelMatrix.identity();
		modelMatrix.translate(bumpPos.x, bumpPos.y, bumpPos.z);
		modelMatrix.rotateY(PI_OVER_2);
		modelMatrix.scale(bumpX, bumpY, bumpZ);
		renderer.renderObject(bumpVAO, modelMatrix, GRAY, true);

		vehicle.render(renderer);
		
		if(renderDebug) {
			dynamicsWorld.debugDrawWorld();
		}
		
		renderer.stop();
	}

	private double lastTime = 0;

	private void layoutGUI() {
		if (ImGui.collapsingHeader("Détails")) {
			ImGui.text("Renderer : " + glGetString(GL_RENDERER));
			int major = glGetInteger(GL_MAJOR_VERSION);
			int minor = glGetInteger(GL_MINOR_VERSION);
			ImGui.text("OpenGL version : " + major + "." + minor);
			double now = glfwGetTime();
			ImGui.text("ips : " + Math.round(1.0 / (now - lastTime)));
			lastTime = now;
			if (ImGui.checkbox("Camera libre", camera.freeCam)) {
				camera.freeCam = !camera.freeCam;
			}
			if (ImGui.checkbox("DebugDrawer", renderDebug)) {
				renderDebug = !renderDebug;
			}
			ImGui.text("Simulation :");
			ImGui.sliderInt("sous-étapes", substeps, 10, 30);
		}
		vehicle.layoutGUI();
	}

	@Override
	protected void postRun() {
		ImPlot.destroyContext(ImPlot.getCurrentContext());

		renderer.destroy();
		vehicle.destroy();
		planeVAO.delete();
		bumpVAO.delete();
		dynamicsWorld.destroy();
	}
}
