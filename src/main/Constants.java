package main;

import javax.vecmath.Vector3f;

public final class Constants {
	public static final Vector3f GRAVITY = new Vector3f(0, -9.81f, 0);
	public static final Vector3f ZERO = new Vector3f(0f, 0f, 0f);
	public static final Vector3f X_AXIS = new Vector3f(1f, 0f, 0f);
	public static final Vector3f Y_AXIS = new Vector3f(0f, 1f, 0f);
	public static final float PI_OVER_2 = (float) (Math.PI / 2f);
	public static final float FRAME_TIME_STEP = 1f / 60f;
}
