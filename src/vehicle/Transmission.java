package vehicle;

import javax.vecmath.Matrix3f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.JacobianEntry;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraintType;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.ScalarUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;

public class Transmission extends TypedConstraint {

	private JacobianEntry[] jac/* [3] */ = new JacobianEntry[] { new JacobianEntry(), new JacobianEntry(),
			new JacobianEntry() }; // 3 orthogonal linear constraints
	private JacobianEntry[] jacAng/* [3] */ = new JacobianEntry[] { new JacobianEntry(), new JacobianEntry(),
			new JacobianEntry() }; // 2 orthogonal angular constraints+ 1 for limit/motor

	private final Transform rbAFrame = new Transform(); // constraint axii. Assumes z is hinge axis.
	private final Transform rbBFrame = new Transform();

	private float motorTargetVelocity;
	private float maxMotorImpulse;

	private float limitSoftness;
	private float biasFactor;
	private float relaxationFactor;

	private float lowerLimit;
	private float upperLimit;

	private float kHinge;

	private float limitSign;
	private float correction;

	private float accLimitImpulse;

	private boolean enableAngularMotor;
	private boolean solveLimit;

	private boolean frontWheelDrive;
	private boolean useDifferential;

	private RigidBody frontLeftWheel;
	private RigidBody frontRightWheel;
	private RigidBody rearLeftWheel;
	private RigidBody rearRightWheel;

	public Transmission(//
			RigidBody frontLeftWheel, //
			RigidBody frontRightWheel, //
			RigidBody rearLeftWheel, //
			RigidBody rearRightWheel) {
		super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, null, null);
		this.frontLeftWheel = frontLeftWheel;
		this.frontRightWheel = frontRightWheel;
		this.rearLeftWheel = rearLeftWheel;
		this.rearRightWheel = rearRightWheel;
		setTransmission(true, false);

		enableAngularMotor = false;

		Vector3f pivotInA = new Vector3f(0, 0, 0);
		Vector3f pivotInB = new Vector3f(0, 0, 0);
		Vector3f axisInA = new Vector3f(1, 0, 0);
		Vector3f axisInB = new Vector3f(1, 0, 0);

		rbAFrame.origin.set(pivotInA);

		// since no frame is given, assume this to be zero angle and just pick rb
		// transform axis
		Vector3f rbAxisA1 = new Vector3f();
		Vector3f rbAxisA2 = new Vector3f();

		Transform centerOfMassA = rbA.getCenterOfMassTransform(new Transform());
		centerOfMassA.basis.getColumn(0, rbAxisA1);
		float projection = axisInA.dot(rbAxisA1);

		if (projection >= 1.0f - BulletGlobals.SIMD_EPSILON) {
			centerOfMassA.basis.getColumn(2, rbAxisA1);
			rbAxisA1.negate();
			centerOfMassA.basis.getColumn(1, rbAxisA2);
		} else if (projection <= -1.0f + BulletGlobals.SIMD_EPSILON) {
			centerOfMassA.basis.getColumn(2, rbAxisA1);
			centerOfMassA.basis.getColumn(1, rbAxisA2);
		} else {
			rbAxisA2.cross(axisInA, rbAxisA1);
			rbAxisA1.cross(rbAxisA2, axisInA);
		}

		rbAFrame.basis.setRow(0, rbAxisA1.x, rbAxisA2.x, axisInA.x);
		rbAFrame.basis.setRow(1, rbAxisA1.y, rbAxisA2.y, axisInA.y);
		rbAFrame.basis.setRow(2, rbAxisA1.z, rbAxisA2.z, axisInA.z);

		Quat4f rotationArc = QuaternionUtil.shortestArcQuat(axisInA, axisInB, new Quat4f());
		Vector3f rbAxisB1 = QuaternionUtil.quatRotate(rotationArc, rbAxisA1, new Vector3f());
		Vector3f rbAxisB2 = new Vector3f();
		rbAxisB2.cross(axisInB, rbAxisB1);

		rbBFrame.origin.set(pivotInB);
		rbBFrame.basis.setRow(0, rbAxisB1.x, rbAxisB2.x, -axisInB.x);
		rbBFrame.basis.setRow(1, rbAxisB1.y, rbAxisB2.y, -axisInB.y);
		rbBFrame.basis.setRow(2, rbAxisB1.z, rbAxisB2.z, -axisInB.z);

		// start with free
		lowerLimit = 0f;
		upperLimit = 0f;
		biasFactor = 0.3f;
		relaxationFactor = 1.0f;
		limitSoftness = 0.9f;
		solveLimit = false;
	}

	public void setTransmission(boolean frontWheelDrive, boolean useDifferential) {
		this.frontWheelDrive = frontWheelDrive;
		this.useDifferential = useDifferential;
		if (frontWheelDrive) {
			rbA = frontLeftWheel;
			rbB = frontRightWheel;
		} else {
			rbA = rearLeftWheel;
			rbB = rearRightWheel;
		}
	}

	@Override
	public void buildJacobian() {
		Vector3f tmp = new Vector3f();
		Matrix3f mat1 = new Matrix3f();
		Matrix3f mat2 = new Matrix3f();

		Transform centerOfMassA = rbA.getCenterOfMassTransform(new Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(new Transform());

		appliedImpulse = 0f;

		// calculate two perpendicular jointAxis, orthogonal to hingeAxis
		// these two jointAxis require equal angular velocities for both bodies

		// this is unused for now, it's a todo
		Vector3f jointAxis0local = new Vector3f();
		Vector3f jointAxis1local = new Vector3f();

		rbAFrame.basis.getColumn(2, tmp);
		TransformUtil.planeSpace1(tmp, jointAxis0local, jointAxis1local);

		// TODO: check this
		// getRigidBodyA().getCenterOfMassTransform().getBasis() *
		// m_rbAFrame.getBasis().getColumn(2);

		Vector3f jointAxis0 = new Vector3f(jointAxis0local);
		centerOfMassA.basis.transform(jointAxis0);

		Vector3f jointAxis1 = new Vector3f(jointAxis1local);
		centerOfMassA.basis.transform(jointAxis1);

		Vector3f hingeAxisWorld = new Vector3f();
		rbAFrame.basis.getColumn(2, hingeAxisWorld);
		centerOfMassA.basis.transform(hingeAxisWorld);

		mat1.transpose(centerOfMassA.basis);
		mat2.transpose(centerOfMassB.basis);
		jacAng[0].init(jointAxis0, mat1, mat2, rbA.getInvInertiaDiagLocal(new Vector3f()),
				rbB.getInvInertiaDiagLocal(new Vector3f()));

		// JAVA NOTE: reused mat1 and mat2, as recomputation is not needed
		jacAng[1].init(jointAxis1, mat1, mat2, rbA.getInvInertiaDiagLocal(new Vector3f()),
				rbB.getInvInertiaDiagLocal(new Vector3f()));

		// JAVA NOTE: reused mat1 and mat2, as recomputation is not needed
		jacAng[2].init(hingeAxisWorld, mat1, mat2, rbA.getInvInertiaDiagLocal(new Vector3f()),
				rbB.getInvInertiaDiagLocal(new Vector3f()));

		// Compute limit information
		float hingeAngle = getHingeAngle();

		// set bias, sign, clear accumulator
		correction = 0f;
		limitSign = 0f;
		solveLimit = false;
		accLimitImpulse = 0f;

		if (!useDifferential) {
			if (hingeAngle <= lowerLimit * limitSoftness) {
				correction = (lowerLimit - hingeAngle);
				limitSign = 1.0f;
				solveLimit = true;
			} else if (hingeAngle >= upperLimit * limitSoftness) {
				correction = upperLimit - hingeAngle;
				limitSign = -1.0f;
				solveLimit = true;
			}
		}

		// Compute K = J*W*J' for hinge axis
		Vector3f axisA = new Vector3f();
		rbAFrame.basis.getColumn(2, axisA);
		centerOfMassA.basis.transform(axisA);

		kHinge = 1.0f / (getRigidBodyA().computeAngularImpulseDenominator(axisA)
				+ getRigidBodyB().computeAngularImpulseDenominator(axisA));
	}

	@Override
	public void solveConstraint(float timeStep) {
		Vector3f tmp = new Vector3f();

		Transform centerOfMassA = rbA.getCenterOfMassTransform(new Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(new Transform());

		Vector3f pivotAInW = new Vector3f(rbAFrame.origin);
		centerOfMassA.transform(pivotAInW);

		Vector3f pivotBInW = new Vector3f(rbBFrame.origin);
		centerOfMassB.transform(pivotBInW);

		// get axes in world space
		Vector3f axisA = new Vector3f();
		rbAFrame.basis.getColumn(2, axisA);
		centerOfMassA.basis.transform(axisA);

		Vector3f axisB = new Vector3f();
		rbBFrame.basis.getColumn(2, axisB);
		centerOfMassB.basis.transform(axisB);

		Vector3f angVelA = getRigidBodyA().getAngularVelocity(new Vector3f());
		Vector3f angVelB = getRigidBodyB().getAngularVelocity(new Vector3f());

		Vector3f angVelAroundHingeAxisA = new Vector3f();
		angVelAroundHingeAxisA.scale(axisA.dot(angVelA), axisA);

		Vector3f angVelAroundHingeAxisB = new Vector3f();
		angVelAroundHingeAxisB.scale(axisB.dot(angVelB), axisB);

		// solve limit
		if (solveLimit) {
			tmp.sub(angVelB, angVelA);
			float amplitude = ((tmp).dot(axisA) * relaxationFactor + correction * (1f / timeStep) * biasFactor)
					* limitSign;

			float impulseMag = amplitude * kHinge;

			// Clamp the accumulated impulse
			float temp = accLimitImpulse;
			accLimitImpulse = Math.max(accLimitImpulse + impulseMag, 0f);
			impulseMag = accLimitImpulse - temp;

			Vector3f impulse = new Vector3f();
			impulse.scale(impulseMag * limitSign, axisA);

			rbA.applyTorqueImpulse(impulse);

			tmp.negate(impulse);
			rbB.applyTorqueImpulse(tmp);
		}

		// apply motor
		if (enableAngularMotor) {
			// todo: add limits too
			Vector3f angularLimit = new Vector3f();
			angularLimit.set(0f, 0f, 0f);

			Vector3f velrel = new Vector3f();
			velrel.add(angVelAroundHingeAxisA, angVelAroundHingeAxisB);
			velrel.scale(0.5f);
			float projRelVel = velrel.dot(axisA);

			float desiredMotorVel = motorTargetVelocity;
			float motor_relvel = desiredMotorVel - projRelVel;

			float unclippedMotorImpulse = kHinge * motor_relvel;

//			 todo: should clip against accumulated impulse
			float clippedMotorImpulse = unclippedMotorImpulse > maxMotorImpulse ? maxMotorImpulse
					: unclippedMotorImpulse;
			clippedMotorImpulse = clippedMotorImpulse < -maxMotorImpulse ? -maxMotorImpulse : clippedMotorImpulse;
			Vector3f motorImp = new Vector3f();
			motorImp.scale(clippedMotorImpulse, axisA);

			tmp.add(motorImp, angularLimit);
			rbA.applyTorqueImpulse(tmp);

//			tmp.negate(motorImp);
//			tmp.sub(angularLimit);
			rbB.applyTorqueImpulse(tmp);
		}
	}

	public float getHingeAngle() {
		Transform centerOfMassA = rbA.getCenterOfMassTransform(new Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(new Transform());

		Vector3f refAxis0 = new Vector3f();
		rbAFrame.basis.getColumn(0, refAxis0);
		centerOfMassA.basis.transform(refAxis0);

		Vector3f refAxis1 = new Vector3f();
		rbAFrame.basis.getColumn(1, refAxis1);
		centerOfMassA.basis.transform(refAxis1);

		Vector3f swingAxis = new Vector3f();
		rbBFrame.basis.getColumn(1, swingAxis);
		centerOfMassB.basis.transform(swingAxis);

		return ScalarUtil.atan2Fast(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
	}

	public void enableAngularMotor(boolean enableMotor, float targetVelocity, float maxMotorImpulse) {
		this.enableAngularMotor = enableMotor;
		this.motorTargetVelocity = targetVelocity;
		this.maxMotorImpulse = maxMotorImpulse;
	}

	public void setBodies(RigidBody rbA, RigidBody rbB) {
		this.rbA = rbA;
		this.rbB = rbB;
	}

	public void setLimit(float low, float high) {
		setLimit(low, high, 0.9f, 0.3f, 1.0f);
	}

	public void setLimit(float low, float high, float _softness, float _biasFactor, float _relaxationFactor) {
		lowerLimit = low;
		upperLimit = high;

		limitSoftness = _softness;
		biasFactor = _biasFactor;
		relaxationFactor = _relaxationFactor;
	}

	public float getLowerLimit() {
		return lowerLimit;
	}

	public float getUpperLimit() {
		return upperLimit;
	}

	public Transform getAFrame(Transform out) {
		out.set(rbAFrame);
		return out;
	}

	public Transform getBFrame(Transform out) {
		out.set(rbBFrame);
		return out;
	}

	public boolean getSolveLimit() {
		return solveLimit;
	}

	public float getLimitSign() {
		return limitSign;
	}

	public boolean getEnableAngularMotor() {
		return enableAngularMotor;
	}

	public float getMotorTargetVelocity() {
		return motorTargetVelocity;
	}

	public float getMaxMotorImpulse() {
		return maxMotorImpulse;
	}

	public boolean getUseDifferential() {
		return useDifferential;
	}

	public boolean getFrontWheelDrive() {
		return frontWheelDrive;
	}

}
