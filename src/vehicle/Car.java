package vehicle;

import static main.Renderer.*;
import static main.Constants.*;
import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL15C.GL_STATIC_DRAW;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import org.joml.Quaternionf;
import org.joml.Vector2f;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CylinderShapeX;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.HingeConstraint;
import com.bulletphysics.dynamics.constraintsolver.SliderConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraintType;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.ScalarUtil;
import com.bulletphysics.linearmath.Transform;

import imgui.ImGui;
import imgui.extension.implot.ImPlot;
import imgui.flag.ImGuiCond;
import imgui.type.ImInt;
import imgui.type.ImString;
import main.Renderer;
import utils.ModelLoader;
import utils.VAO;

public class Car {
	private static final float motorStrength = 20f;
	private static final float brakeStrength = 40f;
	private static final float padding = 0.01f;
	private static final float wheelR = 0.446f / 2f;
	private static final float wheelMass = 8.3f;

	private static final float chassisX = 1.040f / 2f, chassisY = 0.5f / 2f, chassisZ = 2.589f / 2f;
	private static final float carMass = 1200f;
	private static final float maxSteerAngle = (float) Math.toRadians(40);
	private static final float steeringSpeed = 1.5f;
	private static final int numWheelPosition = 240;
	private static final int numOmegaValues = 240;
	private static final int numCenterPosition = 3000;

	private static final Vector3f CHASSISPOSi = new Vector3f(0, chassisY + wheelR, 0);
	private static final Vector3f FLPOSi = new Vector3f(+chassisX + padding + wheelR, wheelR, chassisZ);
	private static final Vector3f FRPOSi = new Vector3f(-chassisX - padding - wheelR, wheelR, chassisZ);
	private static final Vector3f RLPOSi = new Vector3f(+chassisX + padding + wheelR, wheelR, -chassisZ);
	private static final Vector3f RRPOSi = new Vector3f(-chassisX - padding - wheelR, wheelR, -chassisZ);

	private static final byte FORWARD_MASK = 0b00000001;
	private static final byte BACKWARD_MASK = 0b00000010;
	private static final byte LEFT_MASK = 0b00000100;
	private static final byte RIGHT_MASK = 0b00001000;
	private static final byte BRAKING_MASK = 0b00010000;
	private static final byte REDIRECT_MASK = 0b00100000;
	private static final byte FRONT_WHEEL_DRIVE_MASK = 0b01000000;
	private static final byte USE_DIFF_MASK = (byte) 0b10000000;

	private final DynamicsWorld dynamicsWorld;
	private float timeStep;

	private Transmission transmission;

	private RigidBody chassis;
	private RigidBody frontLeftWheel;
	private RigidBody frontRightWheel;
	private RigidBody rearLeftWheel;
	private RigidBody rearRightWheel;

	private SteeringConstraint constraintFL;
	private SteeringConstraint constraintFR;
	private HingeConstraint constraintRL;
	private HingeConstraint constraintRR;

	private float[] wheelPosFront = new float[numWheelPosition * 3 * 2];
	private float[] wheelPosRear = new float[numWheelPosition * 3 * 2];
	private float[] centerPos = new float[numCenterPosition * 3];

	private Float[] FLOmega = new Float[numOmegaValues];
	private Float[] FROmega = new Float[numOmegaValues];
	private Float[] RLOmega = new Float[numOmegaValues];
	private Float[] RROmega = new Float[numOmegaValues];

	private boolean showTrajectories = false;
	private boolean showWheelPos = false;
	private boolean redirectWheels = false;
	private boolean showTireOmega = false;

	private int insertCenterPosIndex = 0;
	private int insertWheelPosIndex = 0;
	private int frame = 0;

	private float steering = 0f;
	private float distance;
	private float centerX;
	private float centerZ;

	private VAO chassisVAO;
	private VAO wheelVAO;
	private VAO wheelPosVAOFront;
	private VAO wheelPosVAORear;
	private VAO centerPosVAO;
	private VAO circleVAO;
	private VAO croixVAO;

	private final long window;

	// écriture
	private ImString outputPath = new ImString(50);
	private boolean recordingManoeuver = false;
	private FileOutputStream outputStream;
	private byte controlsInfo = 0;

	private ImString resultsPath = new ImString(50);
	private boolean savingResults = false;
	private FileWriter results;

	// lecture
	private ImInt selectedFile = new ImInt(0);
	private boolean playingManoeuver = false;
	private int numberOfFrames;
	private FileInputStream inputStream;

	private boolean inverseUSE_DIFF = false;

	public Car(long window, DynamicsWorld dynamicsWorld) {
		this.window = window;
		this.dynamicsWorld = dynamicsWorld;
		initCar();
		initVAOsAndArrays();
	}

	private static Transform translation(Vector3f translation) {
		return new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), translation, 1.0f));
	}

	private void initCar() {
		// le chassis de la voiture
		CollisionShape shape = new BoxShape(new Vector3f(chassisX, chassisY, chassisZ));
		MotionState state0 = new DefaultMotionState(translation(CHASSISPOSi));//
		Vector3f inertia = new Vector3f();
		shape.calculateLocalInertia(carMass, inertia);

		RigidBodyConstructionInfo boxRBInfo = new RigidBodyConstructionInfo(carMass, state0, shape, inertia);
		boxRBInfo.restitution = 0f;
		boxRBInfo.friction = 0.8f;

		chassis = new RigidBody(boxRBInfo);
		chassis.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
		dynamicsWorld.addRigidBody(chassis);

		// les roues et les contraintes
//		CollisionShape wheelShape = new CylinderShapeX(new Vector3f(wheelR, wheelR, wheelR));
		CollisionShape wheelShape = new SphereShape(wheelR); // meileure stabilité avec des sphères plutôt qu'avec des
																// cylindres
		frontLeftWheel = createWheel(wheelShape, FLPOSi);
		frontRightWheel = createWheel(wheelShape, FRPOSi);
		rearLeftWheel = createWheel(wheelShape, RLPOSi);
		rearRightWheel = createWheel(wheelShape, RRPOSi);

		dynamicsWorld.addRigidBody(frontLeftWheel);
		dynamicsWorld.addRigidBody(frontRightWheel);
		dynamicsWorld.addRigidBody(rearLeftWheel);
		dynamicsWorld.addRigidBody(rearRightWheel);

		constraintFL = new SteeringConstraint(chassis, frontLeftWheel, // A, B
				new Vector3f(+chassisX + padding + wheelR, -chassisY, chassisZ), // pivot in A
				ZERO, // pivot in B
				X_AXIS, // axis in A
				X_AXIS);// axis in B

		constraintFR = new SteeringConstraint(chassis, frontRightWheel, // A, B
				new Vector3f(-chassisX - padding - wheelR, -chassisY, chassisZ), // pivot in A
				ZERO, // pivot in B
				X_AXIS, // axis in A
				X_AXIS);// axis in B

		constraintRL = new HingeConstraint(chassis, rearLeftWheel, // A, B
				new Vector3f(+chassisX + padding + wheelR, -chassisY, -chassisZ), // pivot in A
				ZERO, // pivot in B
				X_AXIS, // axis in A
				X_AXIS);// axis in B

		constraintRR = new HingeConstraint(chassis, rearRightWheel, // A, B
				new Vector3f(-chassisX - padding - wheelR, -chassisY, -chassisZ), // pivot in A
				ZERO, // pivot in B
				X_AXIS, // axis in A
				X_AXIS);// axis in B

//		constraintFL.setAngularOnly(true);
//		constraintFR.setAngularOnly(true);
//		constraintRL.setAngularOnly(true);
//		constraintRR.setAngularOnly(true);

		dynamicsWorld.addConstraint(constraintFL);
		dynamicsWorld.addConstraint(constraintFR);
		dynamicsWorld.addConstraint(constraintRL);
		dynamicsWorld.addConstraint(constraintRR);

		transmission = new Transmission(frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel);
		dynamicsWorld.addConstraint(transmission);
	}

	private void initVAOsAndArrays() {
		chassisVAO = ModelLoader.load("res/models/cube.obj", 0, true);
		wheelVAO = ModelLoader.load("res/models/ico_sphere.obj", 0, true);
		float[] circle = new float[3 * 1000];
		for (int i = 0; i < 1000; i++) {
			circle[i * 3 + 0] = (float) Math.cos(i * (Math.PI / 500.0));
			circle[i * 3 + 2] = (float) Math.sin(i * (Math.PI / 500.0));
		}

		circleVAO = new VAO();
		circleVAO.bind();
		circleVAO.createFloatAttribute(0, circle, 3, 0, GL_STATIC_DRAW);
		circleVAO.unbind();

		float[] croix = new float[] { -1, 0, 0, +1, 0, 0, 0, 0, -1, 0, 0, +1 };
		croixVAO = new VAO();
		croixVAO.bind();
		croixVAO.createFloatAttribute(0, croix, 3, 0, GL_STATIC_DRAW);
		croixVAO.unbind();

		Arrays.fill(FLOmega, 0f);
		Arrays.fill(FROmega, 0f);
		Arrays.fill(RLOmega, 0f);
		Arrays.fill(RROmega, 0f);
	}

	private RigidBody createWheel(CollisionShape shape, Vector3f origin) {
		MotionState state0 = new DefaultMotionState(translation(origin));//
		float m = wheelMass;
		float r = wheelR;
		float J = (2f / 3f) * m * r * r;
		Vector3f inertia = new Vector3f(J, J, J);
		RigidBodyConstructionInfo wheelRBInfo = new RigidBodyConstructionInfo(wheelMass, state0, shape, inertia);
		wheelRBInfo.angularDamping = 0.08f;
		wheelRBInfo.restitution = 0.2f;
		wheelRBInfo.friction = 1f;
		RigidBody wheel = new RigidBody(wheelRBInfo);
		wheel.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
		return wheel;
	}

	/**
	 * Remet toute la simulation à zéro
	 */
	private void resetVehicle() {
		frame = 0;

		chassis.clearForces();
		chassis.setLinearVelocity(ZERO);
		chassis.setAngularVelocity(ZERO);
		chassis.setWorldTransform(translation(CHASSISPOSi));

		frontLeftWheel.clearForces();
		frontLeftWheel.setLinearVelocity(ZERO);
		frontLeftWheel.setAngularVelocity(ZERO);
		frontLeftWheel.setWorldTransform(translation(FLPOSi));

		frontRightWheel.clearForces();
		frontRightWheel.setLinearVelocity(ZERO);
		frontRightWheel.setAngularVelocity(ZERO);
		frontRightWheel.setWorldTransform(translation(FRPOSi));

		rearLeftWheel.clearForces();
		rearLeftWheel.setLinearVelocity(ZERO);
		rearLeftWheel.setAngularVelocity(ZERO);
		rearLeftWheel.setWorldTransform(translation(RLPOSi));

		rearRightWheel.clearForces();
		rearRightWheel.setLinearVelocity(ZERO);
		rearRightWheel.setAngularVelocity(ZERO);
		rearRightWheel.setWorldTransform(translation(RRPOSi));

		Arrays.fill(wheelPosFront, 0);
		Arrays.fill(wheelPosRear, 0);
		Arrays.fill(centerPos, 0);
		Arrays.fill(FLOmega, 0f);
		Arrays.fill(FROmega, 0f);
		Arrays.fill(RLOmega, 0f);
		Arrays.fill(RROmega, 0f);

		steering = 0f;

		// important
		dynamicsWorld.getConstraintSolver().reset();
	}

	private boolean forward() {
		if (playingManoeuver) {
			return (controlsInfo & FORWARD_MASK) != 0;
		}

		boolean press = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
		if (recordingManoeuver && press) {
			controlsInfo |= FORWARD_MASK;
		}
		return press;
	}

	private boolean backward() {
		if (playingManoeuver) {
			return (controlsInfo & BACKWARD_MASK) != 0;
		}

		boolean press = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
		if (recordingManoeuver && press) {
			controlsInfo |= BACKWARD_MASK;
		}
		return press;
	}

	private boolean left() {
		if (playingManoeuver) {
			return (controlsInfo & LEFT_MASK) != 0;
		}

		boolean press = glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS;
		if (recordingManoeuver && press) {
			controlsInfo |= LEFT_MASK;
		}
		return press;
	}

	private boolean right() {
		if (playingManoeuver) {
			return (controlsInfo & RIGHT_MASK) != 0;
		}

		boolean press = glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS;
		if (recordingManoeuver && press) {
			controlsInfo |= RIGHT_MASK;
		}
		return press;
	}

	private boolean breaking() {
		if (playingManoeuver) {
			return (controlsInfo & BRAKING_MASK) != 0;
		}

		boolean press = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
		if (recordingManoeuver && press) {
			controlsInfo |= BRAKING_MASK;
		}
		return press;
	}

	/**
	 * met à jour l'angle de braquage des roues
	 */
	private void updateSteering() {
		float factor = (float) Math.pow(Math.min(1f, 1f / chassis.getLinearVelocity(new Vector3f()).length()), 0.6);

		if (left()) {
			steering -= steeringSpeed * factor * FRAME_TIME_STEP;
		} else if (right()) {
			steering += steeringSpeed * factor * FRAME_TIME_STEP;
		} else if (redirectWheels) {
			if (Math.abs(steering) < 0.01f) {
				steering = 0f;
			} else {
				steering -= Math.copySign(steeringSpeed * 0.2f * FRAME_TIME_STEP, steering);
			}
		}

		if (Math.abs(steering) > maxSteerAngle) {
			steering = Math.copySign(maxSteerAngle, steering);
		}

		double L = chassisZ * 2.0;
		double T = chassisX * 2.0 + 2.0 * padding + 2.0 * wheelR;

		double deltaf = Math.abs(steering);
		double R = L / Math.tan(deltaf);
		double dOut = R + T * 0.5;
		double dIn = R - T * 0.5;
		double deltaOut = Math.atan(L / dOut);
		double deltaIn = Math.atan(L / dIn);

		Vector3f axisInA = new Vector3f();
		double steeringLeft, steeringRight;
		if (steering <= 0) {
			steeringLeft = -deltaIn;
			steeringRight = -deltaOut;
		} else {
			steeringLeft = deltaOut;
			steeringRight = deltaIn;
		}
		distance = (float) R;

		axisInA.set((float) Math.cos(steeringLeft), 0, (float) Math.sin(steeringLeft));
		constraintFL.setPivotsAndAxis(//
				new Vector3f(+chassisX + padding + wheelR, -chassisY, chassisZ), // pivot in A
				ZERO, // pivot in B
				axisInA, // axis in A
				X_AXIS);// axis in B

		axisInA.set((float) Math.cos(steeringRight), 0, (float) Math.sin(steeringRight));
		constraintFR.setPivotsAndAxis(//
				new Vector3f(-chassisX - padding - wheelR, -chassisY, chassisZ), // pivot in A
				ZERO, // pivot in B
				axisInA, // axis in A
				X_AXIS);// axis in B

	}

	private void updateBreaking() {
		boolean breaking = breaking();
		constraintFL.enableAngularMotor(breaking, 0, brakeStrength * timeStep);
		constraintFR.enableAngularMotor(breaking, 0, brakeStrength * timeStep);
		constraintRL.enableAngularMotor(breaking, 0, brakeStrength * timeStep);
		constraintRR.enableAngularMotor(breaking, 0, brakeStrength * timeStep);
	}

	private void updateMotor() {
		boolean forward = forward();
		boolean backward = backward();
		transmission.enableAngularMotor(forward || backward, forward ? 150f : backward ? -150f : 0f, motorStrength * timeStep);
	}

	public void update(float timeStep) {
		this.timeStep = timeStep;
		controlsInfo = 0;
		if (playingManoeuver) {
			try {
				int nextByte = inputStream.read();
				if (nextByte == -1) {
					playingManoeuver = false;
					inputStream.close();
					inputStream = null;
					if (savingResults) {
						savingResults = false;
						results.close();
						results = null;
					}
				} else {
					controlsInfo = (byte) nextByte;
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		if (playingManoeuver) {
			redirectWheels = (controlsInfo & REDIRECT_MASK) != 0;
			boolean frontWheelDrive = (controlsInfo & FRONT_WHEEL_DRIVE_MASK) != 0;
			boolean useDifferential = (controlsInfo & USE_DIFF_MASK) != 0;
			if (inverseUSE_DIFF) {
				useDifferential = !useDifferential;
			}
			transmission.setTransmission(frontWheelDrive, useDifferential);
		}

		updateSteering();
		updateMotor();
		updateBreaking();

		if (recordingManoeuver) {
			if (redirectWheels) {
				controlsInfo |= REDIRECT_MASK;
			}
			if (transmission.getFrontWheelDrive()) {
				controlsInfo |= FRONT_WHEEL_DRIVE_MASK;
			}
			if (transmission.getUseDifferential()) {
				controlsInfo |= USE_DIFF_MASK;
			}

			try {
				outputStream.write(controlsInfo);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		Vector3f p0 = frontLeftWheel.getWorldTransform(new Transform()).origin;
		Vector3f p1 = frontRightWheel.getWorldTransform(new Transform()).origin;
		Vector3f p2 = rearLeftWheel.getWorldTransform(new Transform()).origin;
		Vector3f p3 = rearRightWheel.getWorldTransform(new Transform()).origin;
		org.joml.Vector2f center = new org.joml.Vector2f(p2.x, p2.z).sub(p3.x, p3.z)
				.normalize(Math.copySign(distance, -steering)).add((p2.x + p3.x) * 0.5f, (p2.z + p3.z) * 0.5f);
		centerX = center.x;
		centerZ = center.y;

		if (playingManoeuver && savingResults) {
			try {
				results.write(Float.toString(frame / 60f));
				results.write(",");
				results.write(Float.toString(p0.x));
				results.write(",");
				results.write(Float.toString(p0.z));
				results.write(",");

				results.write(Float.toString(p1.x));
				results.write(",");
				results.write(Float.toString(p1.z));
				results.write(",");

				results.write(Float.toString(p2.x));
				results.write(",");
				results.write(Float.toString(p2.z));
				results.write(",");

				results.write(Float.toString(p3.x));
				results.write(",");
				results.write(Float.toString(p3.z));

				results.write("\n");
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		centerPos[(insertCenterPosIndex * 3 + 0) % numCenterPosition] = centerX;
		centerPos[(insertCenterPosIndex * 3 + 2) % numCenterPosition] = centerZ;
		insertCenterPosIndex++;

		if (centerPosVAO != null) {
			centerPosVAO.delete();
		}
		centerPosVAO = new VAO();
		centerPosVAO.bind();
		centerPosVAO.createFloatAttribute(0, centerPos, 3, 0, GL_STATIC_DRAW);
		centerPosVAO.unbind();

		if (frame % 5 == 0) {
			wheelPosFront[(insertWheelPosIndex * 3 * 2 + 0) % (numWheelPosition * 6)] = p0.x;
			wheelPosFront[(insertWheelPosIndex * 3 * 2 + 1) % (numWheelPosition * 6)] = 0.05f;
			wheelPosFront[(insertWheelPosIndex * 3 * 2 + 2) % (numWheelPosition * 6)] = p0.z;

			wheelPosFront[(insertWheelPosIndex * 3 * 2 + 3) % (numWheelPosition * 6)] = p1.x;
			wheelPosFront[(insertWheelPosIndex * 3 * 2 + 4) % (numWheelPosition * 6)] = 0.05f;
			wheelPosFront[(insertWheelPosIndex * 3 * 2 + 5) % (numWheelPosition * 6)] = p1.z;

			wheelPosRear[(insertWheelPosIndex * 3 * 2 + 0) % (numWheelPosition * 6)] = p2.x;
			wheelPosRear[(insertWheelPosIndex * 3 * 2 + 1) % (numWheelPosition * 6)] = 0.05f;
			wheelPosRear[(insertWheelPosIndex * 3 * 2 + 2) % (numWheelPosition * 6)] = p2.z;

			wheelPosRear[(insertWheelPosIndex * 3 * 2 + 3) % (numWheelPosition * 6)] = p3.x;
			wheelPosRear[(insertWheelPosIndex * 3 * 2 + 4) % (numWheelPosition * 6)] = 0.05f;
			wheelPosRear[(insertWheelPosIndex * 3 * 2 + 5) % (numWheelPosition * 6)] = p3.z;

			insertWheelPosIndex++;

			if (wheelPosVAOFront != null) {
				wheelPosVAOFront.delete();
			}
			wheelPosVAOFront = new VAO();
			wheelPosVAOFront.bind();
			wheelPosVAOFront.createFloatAttribute(0, wheelPosFront, 3, 0, GL_STATIC_DRAW);
			wheelPosVAOFront.unbind();

			if (wheelPosVAORear != null) {
				wheelPosVAORear.delete();
			}
			wheelPosVAORear = new VAO();
			wheelPosVAORear.bind();
			wheelPosVAORear.createFloatAttribute(0, wheelPosRear, 3, 0, GL_STATIC_DRAW);
			wheelPosVAORear.unbind();
		}

		// important
		frame++;
	}

	public org.joml.Vector3f getCarPosition() {
		org.joml.Vector3f res = new org.joml.Vector3f();
		Transform transform = new Transform();
		chassis.getMotionState().getWorldTransform(transform);
		res.set(transform.origin.x, transform.origin.y, transform.origin.z);
		return res;
	}

	public Quaternionf getCarDirection() {
		Transform transform = new Transform();
		chassis.getMotionState().getWorldTransform(transform);
		Quat4f quat = new Quat4f();
		transform.getRotation(quat);
		return new Quaternionf(quat.x, quat.y, quat.z, quat.w);
	}

	private void transformModelMatrix(org.joml.Matrix4f mat, RigidBody body) {
		Transform transform = new Transform();
		body.getMotionState().getWorldTransform(transform);
		org.joml.Vector3f translation = new org.joml.Vector3f();
		translation.x = transform.origin.x;
		translation.y = transform.origin.y;
		translation.z = transform.origin.z;

		Quat4f quat = new Quat4f();
		transform.getRotation(quat);
		Quaternionf rotation = new Quaternionf(quat.x, quat.y, quat.z, quat.w);
		mat.translationRotate(translation, rotation);
	}

	private void renderCar(Renderer renderer, org.joml.Matrix4f modelMatrix) {
		transformModelMatrix(modelMatrix, chassis);
		modelMatrix.scale(chassisX, chassisY, chassisZ);
		renderer.renderObject(chassisVAO, modelMatrix, GREEN, true);

		org.joml.Vector3f color;

		transformModelMatrix(modelMatrix, frontLeftWheel);
//		modelMatrix.rotateY(PI_OVER_2);
		modelMatrix.scale(wheelR);
		color = transmission.getFrontWheelDrive() ? BLUE : RED;
		renderer.renderObject(wheelVAO, modelMatrix, color, true);

		transformModelMatrix(modelMatrix, frontRightWheel);
//		modelMatrix.rotateY(PI_OVER_2);
		modelMatrix.scale(wheelR);
		color = transmission.getFrontWheelDrive() ? BLUE : RED;
		renderer.renderObject(wheelVAO, modelMatrix, color, true);

		transformModelMatrix(modelMatrix, rearLeftWheel);
//		modelMatrix.rotateY(PI_OVER_2);
		modelMatrix.scale(wheelR);
		color = transmission.getFrontWheelDrive() ? RED : BLUE;
		renderer.renderObject(wheelVAO, modelMatrix, color, true);

		transformModelMatrix(modelMatrix, rearRightWheel);
//		modelMatrix.rotateY(PI_OVER_2);
		modelMatrix.scale(wheelR);
		color = transmission.getFrontWheelDrive() ? RED : BLUE;
		renderer.renderObject(wheelVAO, modelMatrix, color, true);
	}

	private void renderInfo(Renderer renderer, org.joml.Matrix4f modelMatrix) {
		if (showTrajectories) {
			if (centerPosVAO != null) {
				modelMatrix.identity();
				renderer.renderPointCloud(centerPosVAO, numCenterPosition, modelMatrix, RED, 1f);
			}

			Vector3f RLP = rearLeftWheel.getCenterOfMassPosition(new Vector3f());
			Vector3f RRP = rearRightWheel.getCenterOfMassPosition(new Vector3f());

			modelMatrix.identity();
			modelMatrix.translate(centerX, 0.01f, centerZ);
			modelMatrix.scale(Vector2f.distance(centerX, centerZ, RLP.x, RLP.z));
			renderer.renderLines(circleVAO, 1000, modelMatrix, RED, true);

			modelMatrix.identity();
			modelMatrix.translate(centerX, 0.01f, centerZ);
			modelMatrix.scale(Vector2f.distance(centerX, centerZ, RRP.x, RRP.z));
			renderer.renderLines(circleVAO, 1000, modelMatrix, RED, true);

			Vector3f FLP = frontLeftWheel.getCenterOfMassPosition(new Vector3f());
			Vector3f FRP = frontRightWheel.getCenterOfMassPosition(new Vector3f());

			float distL = Vector2f.distance(centerX, centerZ, FLP.x, FLP.z);
			float distR = Vector2f.distance(centerX, centerZ, FRP.x, FRP.z);

			modelMatrix.identity();
			modelMatrix.translate(centerX, 0.01f, centerZ);
			modelMatrix.scale(distL);
			renderer.renderLines(circleVAO, 1000, modelMatrix, BLUE, true);

			modelMatrix.identity();
			modelMatrix.translate(centerX, 0.01f, centerZ);
			modelMatrix.scale(distR);
			renderer.renderLines(circleVAO, 1000, modelMatrix, BLUE, true);

			modelMatrix.identity();
			modelMatrix.translate(centerX, 0.01f, centerZ);
			modelMatrix.scale(0.1f);
			renderer.renderLines(croixVAO, 4, modelMatrix, RED, false);
		}

		if (showWheelPos) {
			if (wheelPosVAOFront != null) {
				modelMatrix.identity();
				renderer.renderPointCloud(wheelPosVAOFront, numWheelPosition * 4, modelMatrix, BLUE);
			}

			if (wheelPosVAORear != null) {
				modelMatrix.identity();
				renderer.renderPointCloud(wheelPosVAORear, numWheelPosition * 4, modelMatrix, RED);
			}
		}

	}

	public void render(Renderer renderer) {
		org.joml.Matrix4f modelMatrix = new org.joml.Matrix4f();
		renderCar(renderer, modelMatrix);
		renderInfo(renderer, modelMatrix);
	}

	private Vector3f getAngularVelocity(RigidBody body) {
		Vector3f omega3F = body.getAngularVelocity(new Vector3f());
		Transform frameOfRef = body.getWorldTransform(new Transform());
		Matrix3f rotation = new Matrix3f(frameOfRef.basis);
		rotation.invert();
		rotation.transform(omega3F);
		return omega3F;
	}

	private static final Float[] timeAxis = new Float[numOmegaValues];
	static {
		for (int i = 0; i < timeAxis.length; i++) {
			timeAxis[i] = Float.valueOf(i);
		}
	}

	private void layoutOmegaValues(Float[] values, RigidBody wheel, String name) {
		System.arraycopy(values, 1, values, 0, numOmegaValues - 1);
		values[numOmegaValues - 1] = getAngularVelocity(wheel).x;
		ImPlot.plotLine(name, timeAxis, values);
	}

	public void layoutGUI() {
		if (ImGui.collapsingHeader("Véhicule")) {
			layoutGUI1();
		}

		if (ImGui.collapsingHeader("Enregistrement")) {
			layoutGUI2();
		}
	}

	private void layoutGUI1() {
		if (ImGui.button("Remettre à zero")) {
			resetVehicle();
		}
		ImGui.text("Vitesse (km/h) : " + Math.round(chassis.getLinearVelocity(new Vector3f()).length() * 3.6f));
		ImGui.text("Angle de braquage : " + (Math.round(Math.toDegrees(steering) * 100.0) / 100.0) + "°");
		ImGui.beginDisabled(playingManoeuver);
		if (ImGui.checkbox("Rediriger les roues", redirectWheels)) {
			redirectWheels = !redirectWheels;
		}
		ImGui.endDisabled();
		if (ImGui.checkbox("Trajectoires des roues", showTrajectories)) {
			showTrajectories = !showTrajectories;
		}
		if (ImGui.checkbox("Traces des roues", showWheelPos)) {
			showWheelPos = !showWheelPos;
		}
		if (ImGui.checkbox("Vitesse des roues", showTireOmega)) {
			showTireOmega = !showTireOmega;
		}
		if (showTireOmega) {
			ImPlot.setNextPlotLimits(0f, numOmegaValues, -100f, 100f, ImGuiCond.Once);
			if (ImPlot.beginPlot("##NoTitle")) {
				layoutOmegaValues(FLOmega, frontLeftWheel, "FL");
				layoutOmegaValues(FROmega, frontRightWheel, "FR");
				layoutOmegaValues(RLOmega, rearLeftWheel, "RL");
				layoutOmegaValues(RROmega, rearRightWheel, "RR");

				ImPlot.endPlot();
			}
		}

		ImGui.separator();
		ImGui.beginDisabled(playingManoeuver);
		ImGui.text("Transmission :");
		ImGui.sameLine();
		if (ImGui.radioButton("Traction", transmission.getFrontWheelDrive())) {
			transmission.setTransmission(true, transmission.getUseDifferential());
		}
		ImGui.sameLine();
		if (ImGui.radioButton("Propulsion", !transmission.getFrontWheelDrive())) {
			transmission.setTransmission(false, transmission.getUseDifferential());
		}

		if (ImGui.checkbox("Différentiel ouvert", transmission.getUseDifferential())) {
			transmission.setTransmission(transmission.getFrontWheelDrive(), !transmission.getUseDifferential());
		}
		ImGui.endDisabled();
	}

	private void layoutGUI2() {
		ImGui.text("Ecriture :");
		ImGui.beginDisabled(recordingManoeuver || playingManoeuver);
		ImGui.inputText(".bin", outputPath);
		ImGui.endDisabled();
		if (!playingManoeuver) {
			if (!recordingManoeuver) {
				if (ImGui.button("Enregistrer")) {
					String file = "res/save/" + outputPath.get() + ".bin";
					try {
						outputStream = new FileOutputStream(file);
					} catch (FileNotFoundException e) {
						e.printStackTrace();
					}
					recordingManoeuver = true;
					resetVehicle();
				}
			} else {
				ImGui.text("Nombre de frames : " + frame);
				if (ImGui.button("Arrêter l'enregistrement")) {
					recordingManoeuver = false;
					try {
						outputStream.close();
						outputStream = null;
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
			}
		}
		ImGui.separator();
		ImGui.text("Lecture");
		String[] files = new File("res/save").list();
		if (files.length > 0) {
			ImGui.beginDisabled(recordingManoeuver || playingManoeuver);
			ImGui.combo("##file", selectedFile, files);
			if (ImGui.checkbox("Sauvegarder les résultats", savingResults)) {
				savingResults = !savingResults;
			}
			if (savingResults) {
				ImGui.inputText(".txt", resultsPath);
			}
			ImGui.endDisabled();

			if (!recordingManoeuver) {
				if (!playingManoeuver) {
					if (ImGui.checkbox("inverser USE_DIFFERENTIAL", inverseUSE_DIFF)) {
						inverseUSE_DIFF = !inverseUSE_DIFF;
					}
					if (ImGui.button("Lire le fichier")) {
						String file = "res/save/" + files[selectedFile.get()];
						try {
							inputStream = new FileInputStream(file);
							numberOfFrames = (int) Files.size(Path.of(file));
							if (savingResults) {
								results = new FileWriter("res/results/" + resultsPath.get() + ".txt");
								results.write("t,FLx,FLz,FRx,FRz,RLx,RLz,RRx,RRz");
								results.write('\n');
								results.write("s,m,m,m,m,m,m,m,m");
								results.write('\n');
							}
						} catch (IOException e) {
							e.printStackTrace();
						}
						playingManoeuver = true;
						resetVehicle();
					}
				} else {
					ImGui.text("Frame : " + frame + " / " + numberOfFrames);
				}
			}
		}
	}

	public void destroy() {
		chassisVAO.delete();
		wheelVAO.delete();
		circleVAO.delete();
		croixVAO.delete();

		if (wheelPosVAOFront != null) {
			wheelPosVAOFront.delete();
		}
		if (wheelPosVAORear != null) {
			wheelPosVAORear.delete();
		}
		if (centerPosVAO != null) {
			centerPosVAO.delete();
		}

		try {
			if (outputStream != null) {
				outputStream.close();
			}
			if (inputStream != null) {
				inputStream.close();
			}
			if (results != null) {
				results.close();
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

}
