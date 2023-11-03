package com.winteralexander.gdx.ik2d;

import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;

/**
 * A class to represent a FabrikBone2D object.
 * <p>
 * A FabrikBone2D consists of a start location, an end location and a FabrikJoint2D which can constrain
 * the rotation of the bone with regard to either the previous bone in the same chain or with regard
 * to the direction of a bone in another chain which this bone is connected to.
 *
 * @author Al Lansley
 * @version 0.9.2 - 19/06/2019
 */

public class FabrikBone2D implements FabrikBone<Vector2, FabrikJoint2D> {
	/**
	 * mJoint	The joint attached to this FabrikBone2D.
	 * <p>
	 * Each bone has a single FabrikJoint2D which controls the angle to which the bone is
	 * constrained with regard to the previous (i.e. earlier / closer to the base) bone in its chain.
	 * <p>
	 * By default, a joint is not constrained (that is, it is free to rotate up to 180
	 * degrees in a clockwise or anticlockwise direction), however a joint may be
	 * constrained by specifying constraint angles via the
	 * {@link #setClockwiseConstraintDegs(float)} and {@link #setAnticlockwiseConstraintDegs(float)}
	 * methods.
	 * <p>
	 * You might think that surely a bone has two joints, one at the beginning and one at
	 * the end - however, consider an empty chain to which you add a single bone: It has
	 * a joint at its start, around which the bone may rotate (and which it may optionally
	 * be constrained). In this way the single joint which can be considered to be at the
	 * start location of each bone controls the allowable range of motion for that bone alone.
	 */
	private final FabrikJoint2D joint = new FabrikJoint2D();

	/**
	 * startLoc	The start location of this FabrikBone2D object.
	 * <p>
	 * The start location of a bone may only be set through a constructor or via an 'addBone'
	 * or 'addConsecutiveBone' method provided by the {@link FabrikChain2D} class.
	 */
	private final Vector2 startLoc = new Vector2();

	/**
	 * endLoc	The end location of this FabrikBone2D object.
	 * <p>
	 * The end location of a bone may only be set through a constructor or indirectly via an
	 * 'addBone' method provided by the {@link FabrikChain2D} class.
	 */
	private final Vector2 endLoc = new Vector2();

	/**
	 * mLength	The length of this bone from its start location to its end location.
	 * <p>
	 * In the typical usage scenario of a FabrikBone2D the length of the bone remains constant.
	 * <p>
	 * The length may be set explicitly through a value provided to a constructor, or implicitly
	 * when it is calculated as the distance between the {@link #startLoc} and {@link #endLoc}
	 * of a bone.
	 * <p>
	 * Attempting to set a bone length of less than zero, either explicitly or implicitly, will result
	 * in an IllegalArgumentException or
	 */
	private float length;

	/**
	 * mGlobalConstraintUV	The world-space constraint unit-vector of this 2D bone.
	 */
	private final Vector2 mGlobalConstraintUV = new Vector2(1.0f, 0.0f);

	private final Vector2 tmpVec2UV = new Vector2();


	FabrikBone2D() {}

	/**
	 * Constructor to create a new FabrikBone2D from a start and end location as provided by a pair of Vec2fs.
	 * <p>
	 * The {@link #length} property is calculated and set from the provided locations. All other properties
	 * are set to their default values.
	 * <p>
	 * Instantiating a FabrikBone2D with the exact same start and end location, and hence a length of zero,
	 * may result in undefined behaviour.
	 *
	 * @param startLocation The start location of the bone in world space.
	 * @param endLocation   The end location of the bone in world space.
	 */
	public FabrikBone2D(Vector2 startLocation, Vector2 endLocation) {
		startLoc.set(startLocation);
		endLoc.set(endLocation);
		setLength(startLocation.dst(endLocation));
	}

	/**
	 * Constructor to create a new FabrikBone2D from a start and end location as provided by a four floats.
	 * <p>
	 * The {@link #length} property is calculated and set from the provided locations. All other properties
	 * are set to their default values.
	 * <p>
	 * Instantiating a FabrikBone2D with the exact same start and end location, and hence a length of zero,
	 * may result in undefined behaviour.
	 *
	 * @param startX The horizontal start location of the bone in world space.
	 * @param startY The vertical   start location of the bone in world space.
	 * @param endX   The horizontal end   location of the bone in world space.
	 * @param endY   The vertical   end   location of the bone in world space.
	 */
	public FabrikBone2D(float startX, float startY, float endX, float endY) {
		startLoc.set(startX, startY);
		endLoc.set(endX, endY);
		setLength(startLoc.dst(endLoc));
	}

	/**
	 * Constructor to create a new FabrikBone2D from a start location, a direction unit vector and a length.
	 * <p>
	 * A normalised version of the direction unit vector is used to calculate the end location.
	 * <p>
	 * If the provided direction unit vector is zero then an IllegalArgumentException is thrown.
	 * If the provided length argument is less than zero then an IllegalArgumentException is thrown.
	 * <p>
	 * Instantiating a FabrikBone3D with a length of precisely zero may result in undefined behaviour.
	 *
	 * @param startLocation The start location of the bone in world-space.
	 * @param directionUV   The direction unit vector of the bone in world-space.
	 * @param length        The length of the bone in world-space units.
	 */
	public FabrikBone2D(Vector2 startLocation, Vector2 directionUV, float length) {
		// Sanity checking
		// Ensure that the magnitude of this direction unit vector is greater than zero
		if(directionUV.len2() <= 0.0f)
			throw new IllegalArgumentException("Vec2f direction unit vector cannot be zero.");

		// Set the start and end locations
		startLoc.set(startLocation);
		endLoc.set(startLoc).mulAdd(directionUV.nor(), length);

		// Set the bone length via the setLength method rather than directly on the mLength property so that validation is performed
		setLength(length);
	}

	/**
	 * Constructor to create a new FabrikBone2D from a start location, a direction unit vector, a length and
	 * a pair of constraint angles specified in degrees.
	 * <p>
	 * The clockwise and anticlockwise constraint angles can be considered to be relative to the previous bone
	 * in the chain which this bone exists in UNLESS the bone is a basebone (i.e. the first bone in a chain)
	 * in which case, the constraint angles can optionally be made relative to either a world-space direction,
	 * the direction of the bone to which this bone may be connected, or to a direction relative to the coordinate
	 * space of the bone to which this bone may be connected.
	 * <p>
	 * If the direction unit vector argument is zero then an IllegalArgumentException is thrown.
	 * If the length argument is less than zero then an IllegalArgumentException is thrown.
	 * If either the clockwise or anticlockwise constraint angles are outside of the range 0.0f degrees
	 * to 180.0f degrees then an IllegalArgumentException is thrown.
	 * <p>
	 * Instantiating a FabrikBone3D with a length of precisely zero may result in undefined behaviour.
	 *
	 * @param startLocation     The start location of the bone in world-space.
	 * @param directionUV       The direction unit vector of the bone in world-space.
	 * @param length            The length of the bone in world-space units.
	 * @param cwConstraintDegs  The clockwise constraint angle in degrees.
	 * @param acwConstraintDegs The anticlockwise constraint angle in degrees.
	 * @see FabrikChain2D.BaseboneConstraintType2D
	 */
	public FabrikBone2D(Vector2 startLocation,
	                    Vector2 directionUV,
	                    float length,
	                    float cwConstraintDegs,
	                    float acwConstraintDegs) {
		// Set up as per previous constructor - IllegalArgumentExceptions will be thrown for invalid
		// directions or lengths
		this(startLocation, directionUV, length);

		// Set the constraint angles - IllegalArgumentExceptions will be thrown for invalid
		// constraint angles
		setClockwiseConstraintDegs(cwConstraintDegs);
		setAnticlockwiseConstraintDegs(acwConstraintDegs);
	}

	/**
	 * Copy constructor.
	 * <p>
	 * Takes a source FabrikBone2D object and copies all properties into the new FabrikBone2D by
	 * value. Once this is done, there are no shared references between the source and the new
	 * object, and they are exact copies of each other.
	 *
	 * @param source The FabrikBone2D to clone.
	 */
	public FabrikBone2D(FabrikBone2D source) {
		set(source);
	}

	public void set(FabrikBone2D source) {
		startLoc.set(source.startLoc);
		endLoc.set(source.endLoc);
		joint.set(source.joint);
		length = source.length;
		mGlobalConstraintUV.set(source.mGlobalConstraintUV);
	}

	@Override
	public float length() {
		return length;
	}

	@Override
	public Vector2 getStartLocation() {
		return startLoc;
	}

	@Override
	public Vector2 getEndLocation() {
		return endLoc;
	}

	/**
	 * Set the FabrikJoint2D object of this bone.
	 *
	 * @param joint The FabrikJoint2D which this bone should use.
	 */
	public void setJoint(FabrikJoint2D joint) {
		this.joint.set(joint);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public FabrikJoint2D getJoint() {
		return joint;
	}

	/**
	 * Set the clockwise constraint angle of this bone's joint in degrees.
	 * <p>
	 * The valid range of constraint angle is 0.0f degrees to 180.0f degrees inclusive, angles
	 * outside this range are clamped.
	 *
	 * @param angleDegs The clockwise constraint angle specified in degrees.
	 */
	public void setClockwiseConstraintDegs(float angleDegs) {
		joint.setClockwiseConstraintDegs(angleDegs);
	}

	/**
	 * Get the clockwise constraint angle of this bone's joint in degrees.
	 *
	 * @return the clockwise constraint angle in degrees.
	 */
	public float getClockwiseConstraintDegs() {
		return joint.getClockwiseConstraintDegs();
	}

	/**
	 * Set the anticlockwise constraint angle of this bone's joint in degrees.
	 * <p>
	 * The valid range of constraint angle is 0.0f degrees to 180.0f degrees inclusive.
	 * <p>
	 * If a constraint angle outside of this range is provided then an IllegalArgumentException is thrown.
	 *
	 * @param angleDegs The anticlockwise constraint angle specified in degrees.
	 */
	public void setAnticlockwiseConstraintDegs(float angleDegs) {
		joint.setAnticlockwiseConstraintDegs(angleDegs);
	}

	/**
	 * Get the anticlockwise constraint angle of this bone's joint in degrees.
	 *
	 * @return the anticlockwise constraint angle in degrees.
	 */
	public float getAnticlockwiseConstraintDegs() {
		return joint.getAnticlockwiseConstraintDegs();
	}

	/**
	 * Get the direction unit vector between the start location and end location of this bone.
	 * <p>
	 * If the opposite (i.e. end to start) location is required then you can simply negate the provided direction.
	 *
	 * @return The direction unit vector of this bone.
	 */
	public Vector2 getDirectionUV() {
		if(Math.abs(tmpVec2UV.len2() - 1.0f) < MathUtils.FLOAT_ROUNDING_ERROR
				&& tmpVec2UV.dot(endLoc.y - startLoc.y, startLoc.x - endLoc.x) == 0.0f
				&& tmpVec2UV.dot(endLoc.x - startLoc.x, endLoc.y - startLoc.y) > 0.0f)
			return tmpVec2UV;

		tmpVec2UV.set(endLoc).sub(startLoc).nor();
		return tmpVec2UV;
	}

	/**
	 * Get the world-space constraint unit-vector of this bone.
	 *
	 * @return The world-space constraint unit-vector of this bone.
	 */
	public Vector2 getGlobalConstraintUV() {
		return mGlobalConstraintUV;
	}

	/**
	 * Set the world-space constraint unit-vector of this bone.
	 *
	 * @param v The world-space constraint unit vector.
	 */
	public void setGlobalConstraintUV(Vector2 v) {
		this.mGlobalConstraintUV.set(v);
	}

	/**
	 * Return the coordinate system to use for any constraints applied to the joint of this bone.
	 *
	 * @return The coordinate system to use for any constraints applied to the joint of this bone.
	 */
	public FabrikJoint2D.ConstraintCoordinateSystem getJointConstraintCoordinateSystem() {
		return this.joint.getConstraintCoordinateSystem();
	}

	/**
	 * Set the coordinate system to use for any constraints applied to the joint of this bone.
	 *
	 * @param coordSystem The coordinate system to use for any constraints applied to the joint of this bone.
	 */
	public void setJointConstraintCoordinateSystem(FabrikJoint2D.ConstraintCoordinateSystem coordSystem) {
		this.joint.setConstraintCoordinateSystem(coordSystem);
	}

	/**
	 * Return a concise, human readable description of this FabrikBone2D as a String.
	 * <p>
	 * The colour and line-width are not included in this output, but can be queried separately
	 * via the getColour and getLineWidth methods.
	 */
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();

		sb.append("Start joint location : ").append(startLoc).append("\n");
		sb.append("End   joint location : ").append(endLoc).append("\n");
		sb.append("Bone direction       : ").append(getDirectionUV()).append("\n");
		sb.append("Bone length          : ").append(length).append("\n");
		sb.append(joint.toString());

		return sb.toString();
	}

	@Override
	public void setStartLocation(Vector2 location) {
		startLoc.set(location);
	}

	@Override
	public void setEndLocation(Vector2 location) {
		endLoc.set(location);
	}

	/**
	 * Set the length of the bone.
	 * <p>
	 * If the length argument is not greater then zero an IllegalArgumentException is thrown.
	 * If the length argument is precisely zero then
	 *
	 * @param length The value to set on the {@link #length} property.
	 */
	private void setLength(float length) {
		if(length >= 0.0f) {
			this.length = length;
		} else {
			throw new IllegalArgumentException("Bone length must be a positive value.");
		}
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + endLoc.hashCode();
		result = prime * result + joint.hashCode();
		result = prime * result + Float.floatToIntBits(length);
		result = prime * result + startLoc.hashCode();
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if(this == obj) {
			return true;
		}
		if(obj == null) {
			return false;
		}
		if(getClass() != obj.getClass()) {
			return false;
		}
		FabrikBone2D other = (FabrikBone2D)obj;
		return endLoc.equals(other.endLoc)
				&& joint.equals(other.joint)
				&& Float.floatToIntBits(length) == Float.floatToIntBits(other.length)
				&& startLoc.equals(other.startLoc);
	}

} // End of FabrikBone2D class
