package com.winteralexander.gdx.ik2d;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import com.winteralexander.gdx.ik2d.FabrikChain2D.BaseboneConstraintType2D;

/**
 * Class to represent a 2D Inverse Kinematics (IK) chain that can be solved for a given target using the FABRIK algorithm.
 *
 * @author Al Lansley
 * @version 1.2 - 19/06/2019
 */

public class FabrikChain2D implements FabrikChain<FabrikBone2D,
		Vector2,
		FabrikJoint2D,
		BaseboneConstraintType2D> {
	/**
	 * Basebone constraint types.
	 * <p>
	 * Basebone constraints constrain the basebone (i.e. the first bone in a chain) and may be of the following types:
	 * <ul>
	 * <li>NONE - No constraint.</li>
	 * <li>GLOBAL_ABSOLUTE - Constrained about a world-space direction.</li>
	 * <li>LOCAL_RELATIVE - Constrained about the direction of the connected bone.</li>
	 * <li>LOCAL_ABSOLUTE - Constrained about a direction with relative to the direction of the connected bone.</li>
	 * </ul>
	 * <p>
	 * See {@link #setBaseboneConstraintType(BaseboneConstraintType2D)}
	 */
	public enum BaseboneConstraintType2D implements BaseboneConstraintType {
		NONE, GLOBAL_ABSOLUTE, LOCAL_RELATIVE, LOCAL_ABSOLUTE
	}


	/**
	 * The core of a FabrikChain2D is a list of FabrikBone2D objects, where each bone contains a start and end location, and a joint
	 * that stores any rotational constraints.
	 */
	private final Array<FabrikBone2D> chain = new Array<>();

	/**
	 * The distance threshold we must meet in order to consider this FabrikChain2D to be successfully solved for distance.
	 * <p>
	 * When we solve a chain so that the distance between the end effector and target is less than or equal to the distance
	 * threshold, then we consider the chain to be successfully solved and will dynamically abort any further attempts to solve the chain.
	 * <p>
	 * The minimum valid distance threshold is 0.0f, however a slightly higher value should be used to avoid forcing the IK
	 * chain solve process to run repeatedly when an <strong>acceptable</strong> (but not necessarily <em>perfect</em>)
	 * solution is found.
	 * <p>
	 * Although this property is the main criteria used to establish whether or not we have solved a given IK chain, it works
	 * in combination with the {@link #maxIterationAttempts} and {@link #minIterationChange} fields to improve the
	 * performance of the algorithm in situations where we may not be able to solve a given IK chain. Such situations may arise
	 * when bones in the chain are highly constrained, or when the target is further away than the length of a chain which has
	 * a fixed base location.
	 * <p>
	 * The default value is 1.0f.
	 * <p>
	 * See {@link #setSolveDistanceThreshold(float)}
	 */
	private float solveDistanceThreshold = 1.0f;

	/**
	 * Specifies the maximum number of attempts that will be performed in order to solve this FabrikChain2D.
	 * <p>
	 * As FABRIK is an iterative algorithm, after a single solve attempt we may not be within the solve distance threshold for the chain, so must take
	 * the current chain configuration and perform another solve attempt. With a reasonable value for the solve distance threshold (i.e. the distance which
	 * we must solve the chain for in order to consider it solved), it is common to be able to solve a given IK chain within a small number of attempts.
	 * <p>
	 * However, some IK chains cannot be solved, whether this is due to joint constraints, or (in the case of an IK chain with a fixed base location)
	 * where the distance between the base location and the target is greater than the length of the chain itself. In these cases, we may abort
	 * any further attempts to solve the chain after this number of solve attempts. This decreases the execution time of the algorithm, as there
	 * is no point in repeatedly attempting to solve an IK chain which cannot be solved.
	 * <p>
	 * Further, we may find that we are making unacceptably low progress towards solving the chain, then will also abort solving the chain based on the
	 * {@link #minIterationChange} threshold.
	 * <p>
	 * The default value is 15.
	 * <p>
	 * See {@link #setMaxIterationAttempts(int)}
	 */
	private int maxIterationAttempts = 15;

	/**
	 * Specifies the minimum change in the solve distance which must be made from one solve attempt to the next in order for us to believe it
	 * worthwhile to continue making attempts to solve the IK chain.
	 * <p>
	 * As FABRIK is an iterative algorithm, after a single solve attempt we may not be within the solve distance threshold for the chain, so must take
	 * the current chain configuration and perform another solve attempt. With a reasonable value for the solve distance threshold (i.e. the distance which
	 * we must solve the chain for in order to consider it solved), it is common to be able to solve a given IK chain within a small number of attempts.
	 * <p>
	 * However, some IK chains cannot be solved, whether this is due to joint constraints, or (in the case of an IK chain with a fixed base location)
	 * where the distance between the base location and the target is greater than the length of the chain itself. In these cases, we abort attempting
	 * to solve the chain if we are making very little progress towards solving the chain between solve attempts. This decreases the execution time of
	 * the algorithm, as there  is no point in repeatedly attempting to solve an IK chain which cannot be solved.
	 * <p>
	 * This property must be greater than or equal to zero.
	 * <p>
	 * The default value is 0.01f.
	 * <p>
	 * See {@link #setMinIterationChange(float)}
	 */
	private float minIterationChange = 0.01f;

	/**
	 * The chainLength is the combined length of all bones in this FabrikChain2D object.
	 * <p>
	 * When a FabrikBone2D is added or removed from the chain using the addBone/addConsecutiveBone or removeBone methods, then
	 * the chainLength is updated to reflect this.
	 */
	private float chainLength;

	/**
	 * The location of the start joint of the first bone added to the chain.
	 * <p>
	 * By default, FabrikChain2D objects are created with a fixed base location, that is, the start joint of the first bone in the chain is
	 * immovable. As such, the start joint location of the first bone in the chain is stored in this property so that the base of the chain
	 * can be snapped back to this location during the FABRIK solve process.
	 * <p>
	 * Base locations do not <em>have</em> to be fixed - this property can be modified to via the setFixedBaseLocation method.
	 * <p>
	 * See {@link #setFixedBaseMode(boolean)}
	 */
	private final Vector2 baseLocation = new Vector2();

	/**
	 * mFixedBaseMode	Whether this FabrikChain2D has a fixed (i.e. immovable) base location.
	 * <p>
	 * By default, the location of the start joint of the first bone added to the IK chain is considered fixed. This
	 * 'anchors' the base of the chain in place. Optionally, a user may call the setFixedBaseMode method to disable
	 * fixing the base bone start location in place.
	 * <p>
	 * The default value is true (i.e. the basebone of the chain is fxied in place).
	 * <p>
	 * See {@link #setFixedBaseMode(boolean)}
	 */
	private boolean fixedBaseMode = true;

	/**
	 * The type of constraint applied to the basebone of this chain.
	 * <p>
	 * The default value is BaseboneConstraintType2D.NONE.
	 */
	private BaseboneConstraintType2D baseboneConstraintType = BaseboneConstraintType2D.NONE;

	/**
	 * If this chain is connected to a bone in another chain, then this property controls whether
	 * it should connect to the start end location of the host bone.
	 * <p>
	 * The default value is BoneConnectionPoint.END.
	 * <p>
	 * See {@link #setBoneConnectionPoint(BoneConnectionPoint)}
	 * See {@link FabrikStructure2D#connectChain(FabrikChain2D, int, int, BoneConnectionPoint)}
	 */
	private BoneConnectionPoint boneConnectionPoint = BoneConnectionPoint.END;

	/**
	 * The direction around which we should constrain the base bone, as provided to the setBaseboneConstraintUV method.
	 * <p>
	 * See {@link #setBaseboneConstraintUV(Vector2)}
	 */
	private final Vector2 baseboneConstraintUV = new Vector2();

	/**
	 * The (internally used) constraint unit vector when this chain is connected to another chain in either
	 * LOCAL_ABSOLUTE or LOCAL_RELATIVE modes.
	 * <p>
	 * This property cannot be accessed by users and is updated in the FabrikStructure2D.solveForTarget() method.
	 */
	private final Vector2 baseboneRelativeConstraintUV = new Vector2();

	/**
	 * mLastTargetLocation	The last target location for the end effector of this IK chain.
	 * <p>
	 * The last target location is used during the solve attempt and is specified as a property of the chain to avoid
	 * memory allocation at runtime.
	 */
	private final Vector2 lastTargetLocation = new Vector2(Float.MAX_VALUE, Float.MAX_VALUE);

	/**
	 * mLastBaseLocation - The previous base location of the chain from the last solve attempt.
	 * <p>
	 * The default value is Vec2f(Float.MAX_VALUE, Float.MAX_VALUE).
	 */
	private final Vector2 lastBaseLocation = new Vector2(Float.MAX_VALUE, Float.MAX_VALUE);

	/**
	 * mEmbeddedTarget	An embedded target location which can be used to solve this chain.
	 * <p>
	 * Embedded target locations allow structures to be solved for multiple targets (one per chain in the structure)
	 * rather than all chains being solved for the same target. To use embedded targets, the mUseEmbeddedTargets flag
	 * must be true (which is not the default) - this flag can be set via a call to setEmbeddedTarget(true).
	 * <p>
	 * See {@link #setEmbeddedTargetMode(boolean)}
	 */
	private final Vector2 embeddedTarget = new Vector2();

	/**
	 * mUseEmbeddedTarget	Whether or not to use the mEmbeddedTarget location when solving this chain.
	 * <p>
	 * This flag may be toggled by calling the setEmbeddedTargetMode(boolean) method on the chain.
	 * <p>
	 * The default value is false.
	 * <p>
	 * See {@link #setEmbeddedTargetMode(boolean)}
	 */
	private boolean useEmbeddedTarget = false;

	/**
	 * mCurrentSolveDistance	The current distance between the end effector and the target location for this IK chain.
	 * <p>
	 * The current solve distance is updated when an attempt is made to solve IK chain as triggered by a call to the
	 * {@link #solveForTarget(Vector2)} or (@link #solveForTarget(float, float) methods.
	 */
	private float currentSolveDistance = Float.MAX_VALUE;

	/**
	 * The zero-indexed number of the chain this chain is connected to in a FabrikStructure2D.
	 * <p>	 *
	 * If this value is -1 then this chain is not connected to another chain.
	 * <p>
	 * The default value is -1.
	 */
	private int connectedChainNumber = -1;

	/**
	 * The zero-indexed number of the bone that this chain is connected to, if it's connected to another chain at all.
	 * <p>
	 * If the value is -1 then it's not connected to another bone in another chain.
	 * <p>
	 * The default value is -1.
	 */
	private int connectedBoneNumber = -1;

	private final Vector2 tmpDir = new Vector2();
	private final Vector2 tmpOuterDir = new Vector2();
	private final Vector2 tmpInnerDir = new Vector2();
	private final Vector2 tmpConstrainedUV = new Vector2();

	private final Array<FabrikBone2D> tmpStartSolution = new Array<>();
	private final Array<FabrikBone2D> tmpBestSolution = new Array<>();

	public FabrikChain2D() {}

	/**
	 * Copy constructor.
	 *
	 * @param source The FabrikChain2D to use as the basis for this new FabrikChain2D.
	 */
	public FabrikChain2D(FabrikChain2D source) {
		// Force copy by value
		source.cloneChainVector(chain);
		baseLocation.set(source.baseLocation);
		lastTargetLocation.set(source.lastTargetLocation);
		lastBaseLocation.set(source.lastBaseLocation);
		baseboneConstraintUV.set(source.baseboneConstraintUV);
		baseboneRelativeConstraintUV.set(source.baseboneRelativeConstraintUV);
		embeddedTarget.set(source.embeddedTarget);

		// Native copy by value for primitive members
		chainLength = source.chainLength;
		currentSolveDistance = source.currentSolveDistance;
		connectedChainNumber = source.connectedChainNumber;
		connectedBoneNumber = source.connectedBoneNumber;
		baseboneConstraintType = source.baseboneConstraintType;
		boneConnectionPoint = source.boneConnectionPoint;
		useEmbeddedTarget = source.useEmbeddedTarget;
	}

	// ---------- Public Methods ------------

	/**
	 * Add a bone to the end of this IK chain of this FabrikChain2D object.
	 * <p>
	 * This chain's {@link #chainLength} property is updated to take into account the length of the
	 * new bone added to the chain.
	 * <p>
	 * In addition, if the bone being added is the very first bone, then this chain's
	 * {@link #baseLocation} property is set from the start joint location of the bone.
	 *
	 * @param bone The FabrikBone2D object to add to this FabrikChain2D.
	 * @see #chainLength
	 * @see #baseLocation
	 */
	@Override
	public void addBone(FabrikBone2D bone) {
		// Add the new bone to the end of the chain
		chain.add(bone);

		// If this is the basebone...
		if(this.chain.size == 1) {
			// ...then keep a copy of the fixed start location...
			baseLocation.set(bone.getStartLocation());

			// ...and set the basebone constraint UV to be around the bone direction
			baseboneConstraintUV.set(bone.getDirectionUV());
		}

		// ...and update the desired length of the chain (i.e. combined length of all bones)
		updateChainLength();
	}

	/**
	 * Add a constrained bone to the end of this IK chain.
	 * <p>
	 * The constraint angles are relative to the coordinate system of the previous bone in the chain.
	 * <p>
	 * This method can only be used when the IK chain contains a base bone, as without it we do not
	 * have a start location for this bone (i.e. the end location of the previous bone).
	 * <p>
	 * If this method is called on a chain which does not contain a base bone
	 * then a {@link RuntimeException} is thrown.
	 * <p>
	 * Specifying a direction unit vector of zero will result in an IllegalArgumentException being thrown.
	 *
	 * @param directionUV       The initial direction of the new bone
	 * @param length            The length of the new bone
	 * @param clockwiseDegs     The clockwise constraint angle in degrees.
	 * @param anticlockwiseDegs The anti-clockwise constraint angle in degrees.
	 */
	public void addConsecutiveConstrainedBone(Vector2 directionUV, float length, float clockwiseDegs, float anticlockwiseDegs) {
		// Validate the direction unit vector - throws an IllegalArgumentException if it has a magnitude of zero
		// Ensure that the magnitude of this direction unit vector is greater than zero
		if(directionUV.len2() <= 0.0f) {
			throw new IllegalArgumentException("Vec2f direction unit vector cannot be zero.");
		}

		// Validate the length of the bone - throws an IllegalArgumentException if it is not a positive value
		// Ensure that the magnitude of this direction unit vector is not zero
		if(length < 0.0f)
			throw new IllegalArgumentException("Length must be a greater than or equal to zero.");

		// If we have at least one bone already in the chain...
		if(!this.chain.isEmpty()) {
			// Get the end location of the last bone, which will be used as the start location of the new bone
			Vector2 prevBoneEnd = chain.get(this.chain.size - 1).getEndLocation();

			// Add a bone to the end of this IK chain
			addBone(new FabrikBone2D(prevBoneEnd, directionUV.nor(), length, clockwiseDegs, anticlockwiseDegs));
		} else {
			// Attempting to add a relative bone when there is no base bone for it to be relative to?
			throw new RuntimeException("You cannot add the base bone to a chain using this method as it does not provide a start location.");
		}
	}

	/**
	 * Add a unconstrained bone to the end of this IK chain.
	 * <p>
	 * This method can only be used when the IK chain contains a base bone, as without it we do not
	 * have a start location for this bone (i.e. the end location of the previous bone).
	 * <p>
	 * If this method is called on a chain which does not contain a base bone
	 * then a {@link RuntimeException} is thrown.
	 * <p>
	 * Specifying a direction unit vector of zero will result in an IllegalArgumentException being thrown.
	 *
	 * @param directionUV The initial direction of the new bone
	 * @param length      The length of the new bone
	 */
	@Override
	public void addConsecutiveBone(Vector2 directionUV, float length) {
		addConsecutiveConstrainedBone(directionUV, length, 180.0f, 180.0f);
	}

	/**
	 * Add pre-created bone to the end of this IK chain.
	 * <p>
	 * This method can only be used when the IK chain contains a base bone, as without it we do not
	 * have a start location for this bone (i.e. the end location of the previous bone).
	 * <p>
	 * If this method is called on a chain which does not contain a base bone
	 * then a {@link RuntimeException} is thrown.
	 * <p>
	 * Specifying a direction unit vector of zero will result in an IllegalArgumentException being thrown.
	 *
	 * @param bone The bone to add to the end of this chain.
	 */
	@Override
	public void addConsecutiveBone(FabrikBone2D bone) {
		// Validate the direction unit vector - throws an IllegalArgumentException if it has a magnitude of zero
		Vector2 dir = bone.getDirectionUV();
		// Ensure that the magnitude of this direction unit vector is greater than zero
		if(dir.len2() <= 0.0f) {
			throw new IllegalArgumentException("Vec2f direction unit vector cannot be zero.");
		}

		// Validate the length of the bone - throws an IllegalArgumentException if it is not a positive value
		float len = bone.length();
		// Ensure that the magnitude of this direction unit vector is not zero
		if(len < 0.0f) {
			throw new IllegalArgumentException("Length must be a greater than or equal to zero.");
		}

		// If we have at least one bone already in the chain...
		if(!this.chain.isEmpty()) {
			// Get the end location of the last bone, which will be used as the start location of the new bone
			Vector2 prevBoneEnd = chain.get(this.chain.size - 1).getEndLocation();

			bone.setStartLocation(prevBoneEnd);
			bone.getEndLocation().set(prevBoneEnd).mulAdd(dir, len);

			// Add a bone to the end of this IK chain
			addBone(bone);
		} else {// Attempting to add a relative bone when there is no base bone for it to be relative to?
			throw new RuntimeException("You cannot add the base bone to a chain using this method as it does not provide a start location.");
		}
	}

	/**
	 * Return the basebone constraint type of this chain.
	 *
	 * @return The basebone constraint type of this chain.
	 */
	@Override
	public BaseboneConstraintType2D getBaseboneConstraintType() {
		return baseboneConstraintType;
	}

	/**
	 * Return the basebone constraint unit vector.
	 *
	 * @return The basebone constraint unit vector of this FabrikChain2D.
	 */
	@Override
	public Vector2 getBaseboneConstraintUV() {
		return baseboneConstraintUV;
	}

	/**
	 * Return
	 * <p>
	 * Regardless of how many bones are contained in the chain, the base location is always the start location of the first bone in the chain.
	 * <p>
	 * If there are no bones in the chain when this method is called then a RuntimeException is thrown.
	 *
	 * @return The base location of this FabrikChain2D.
	 */
	@Override
	public Vector2 getBaseLocation() {
		if(!this.chain.isEmpty()) {
			return chain.get(0).getStartLocation();
		} else {
			throw new RuntimeException("Cannot get base location as there are zero bones in the chain.");
		}
	}

	/**
	 * Return a bone by its location in the IK chain.
	 * <p>
	 * The base bone is always bone 0, each additional bone increases the number by 1.
	 *
	 * @param boneNumber The number of the bone to return from the Vector of FabrikBone2D objects.
	 * @return The FabrikBone2D at the given location in this chain.
	 */
	@Override
	public FabrikBone2D getBone(int boneNumber) {
		return chain.get(boneNumber);
	}

	/**
	 * Return the bone connection point of this chain.
	 * <p>
	 * The connection point will be either BoneConnectionPoint2D.START or BoneConnectionPoint2D.END.
	 *
	 * @return The bone connection point of this chain.
	 */
	public BoneConnectionPoint getBoneConnectionPoint() {
		return boneConnectionPoint;
	}

	/**
	 * Return the actual IK chain of this FabrikChain2D object,
	 *
	 * @return The IK chain of this FabrikChain2D as a List of FabrikBone2D objects.
	 */
	@Override
	public Array<FabrikBone2D> getChain() {
		return chain;
	}

	/**
	 * Return the current length of this IK chain.
	 * <p>
	 * This method does not dynamically re-calculate the length of the chain - it merely returns the previously
	 * calculated chain length, which gets updated each time a bone is added to the chain.
	 *
	 * @return The length of this IK chain as stored in the mChainLength property.
	 */
	@Override
	public float getChainLength() {
		return chainLength;
	}

	/**
	 * Return the number of the bone in another chain that this FabrikChain2D is connected to.
	 *
	 * @return The connected bone number as stored in the mConnectedBoneNumber property.
	 */
	@Override
	public int getConnectedBoneNumber() {
		return connectedBoneNumber;
	}

	/**
	 * Return the number of the chain in the structure containing this FabrikChain2D that this chain is connected to.
	 *
	 * @return The connected chain number as stored in the mConnectedChainNumber property.
	 */
	@Override
	public int getConnectedChainNumber() {
		return connectedChainNumber;
	}

	/**
	 * Return the location of the end effector in the IK chain.
	 * <p>
	 * Regardless of how many bones are contained in the chain, the end effector is always
	 * the end location of the final bone in the chain.
	 * <p>
	 * If there are no bones in the chain when this method is called then a RuntimeException is thrown.
	 *
	 * @return Vec2f
	 */
	@Override
	public Vector2 getEffectorLocation() {
		if(!this.chain.isEmpty()) {
			return chain.get(this.chain.size - 1).getEndLocation();
		} else {
			throw new RuntimeException("Cannot get effector location as there are zero bones in the chain.");
		}
	}

	/**
	 * Return whether or not this chain uses an embedded target.
	 * <p>
	 * Embedded target mode may be enabled or disabled using setEmbeddededTargetMode(boolean).
	 *
	 * @return whether or not this chain uses an embedded target.
	 */
	@Override
	public boolean getEmbeddedTargetMode() {
		return useEmbeddedTarget;
	}

	/**
	 * Return the embedded target location.
	 *
	 * @return the embedded target location.
	 */
	@Override
	public Vector2 getEmbeddedTarget() {
		return embeddedTarget;
	}

	/**
	 * Return the last target location for which we  attempted to solve this IK chain.
	 * <p>
	 * The last target location and the current target location may not be the same.
	 *
	 * @return The last target location that we attempted to solve this chain for.
	 */
	@Override
	public Vector2 getLastTargetLocation() {
		return lastTargetLocation;
	}

	/**
	 * Return the number of bones in the IK chain.
	 * <p>
	 * Bones may be added to the chain via the addBone or addConsecutiveBone methods.
	 *
	 * @return The number of bones in the FabrikChain2D.
	 */
	@Override
	public int getNumBones() {
		return this.chain.size;
	}

	/**
	 * Remove a bone from this IK chain.
	 * <p>
	 * Bone numbers start at 0, so if there were 3 bones in a chain they would be at locations 0, 1 and 2.
	 * <p>
	 * This chain's {@link #chainLength} property is updated to take into account the new chain length.
	 * <p>
	 * If this chain does not contain a bone at the specified location then an IllegalArgumentException is thrown.
	 *
	 * @param boneNumber The zero-indexed bone to remove from this IK chain.
	 * @see #chainLength
	 * @see #baseLocation
	 */
	@Override
	public void removeBone(int boneNumber) {
		// If the bone number is a bone which exists...
		if(boneNumber < this.chain.size) {
			// ...then remove the bone, decrease the bone count and update the chain length.
			chain.removeIndex(boneNumber);
			updateChainLength();
		} else {
			throw new IllegalArgumentException("Bone " + boneNumber + " does not exist in this chain.");
		}
	}

	/**
	 * Set the basebone constraint type on this chain.
	 *
	 * @param type The BaseboneConstraintType2D to set.
	 */
	public void setBaseboneConstraintType(BaseboneConstraintType2D type) {
		baseboneConstraintType = type;
	}

	/**
	 * Set the constraint unit vector for the basebone of this chain..
	 * <p>
	 * The bone will be constrained about the clockwise and anticlockwise constraint angles with regard to this
	 * direction methods on the FabrikBone2D to be constrained.
	 *
	 * @param constraintUV The direction unit vector to constrain the base bone to.
	 * @see #setBaseboneConstraintType(BaseboneConstraintType2D)
	 * @see FabrikJoint2D#setClockwiseConstraintDegs
	 * @see FabrikJoint2D#setAnticlockwiseConstraintDegs
	 */
	@Override
	public void setBaseboneConstraintUV(Vector2 constraintUV) {
		// Sanity checking
		// Ensure that the magnitude of this direction unit vector is greater than zero
		if(constraintUV.len2() <= 0.0f)
			throw new IllegalArgumentException("Vec2f direction unit vector cannot be zero.");

		baseboneConstraintUV.set(constraintUV).nor();
	}

	/**
	 * Set the base location of this chain.
	 *
	 * @param baseLocation The location to set the mBaseLocation property.
	 * @see #baseLocation
	 */
	public void setBaseLocation(Vector2 baseLocation) {
		this.baseLocation.set(baseLocation);
	}

	/**
	 * Set the bone connection point of this chain.
	 * <p>
	 * When this chain is connected to another chain, it may either connect to the bone in the other chain at
	 * the start (BoneConnectionPoint.START) or end (BoneConnectionPoint.END) of the bone.
	 * <p>
	 * This property is held per-chain rather than per-bone for efficiency.
	 *
	 * @param boneConnectionPoint The BoneConnectionPoint to set on this chain.
	 */
	public void setBoneConnectionPoint(BoneConnectionPoint boneConnectionPoint) {
		this.boneConnectionPoint = boneConnectionPoint;
	}

	/**
	 * Set the List%lt;FabrikBone2D%gt; of this FabrikChain2D to the provided list by reference.
	 *
	 * @param chain The List%lt;FabrikBone2D%gt; of FabrikBone2D objects to assign to this chain.
	 * @see #chain
	 */
	public void setChain(Array<FabrikBone2D> chain) {
		// Assign this chain to be a reference to the chain provided as an argument to this method
		this.chain.clear();
		this.chain.addAll(chain);
	}

	/**
	 * Set which number bone this chain is connected to in another chain in a FabrikStructure2D object.
	 *
	 * @param boneNumber The number of the bone this chain is connected to.
	 */
	void setConnectedBoneNumber(int boneNumber) {
		connectedBoneNumber = boneNumber;
	}

	/**
	 * Set which number chain this chain is connected to in a FabrikStructure2D object.
	 *
	 * @param chainNumber The number of the chain that this chain is connected to.
	 */
	void setConnectedChainNumber(int chainNumber) {
		connectedChainNumber = chainNumber;
	}

	/**
	 * Set the fixed base bone mode for this chain.
	 * <p>
	 * If the base bone is 'fixed' in place, then its start location cannot move unless it is attached
	 * to another chain (which moves). The bone is still allowed to rotate, with or without constraints.
	 * <p>
	 * Specifying a non-fixed base location while this chain is connected to another chain will result in a
	 * RuntimeException being thrown.
	 * <p>
	 * Fixing the base bone's start location in place and constraining to a global absolute direction are
	 * mutually exclusive. Disabling fixed base mode while the chain's constraint type is
	 * BaseBoneConstraintType2D.GLOBAL_ABSOLUTE will result in a RuntimeException being thrown.
	 *
	 * @param value Whether or not to fix the base bone start location in place.
	 */
	@Override
	public void setFixedBaseMode(boolean value) {
		if(!value && connectedChainNumber != -1) {
			throw new RuntimeException("This chain is connected to another chain so must remain in fixed base mode.");
		}

		// We cannot have a freely moving base location AND constrain the base bone - so if we have both
		// enabled then we disable the base bone angle constraint here.
		if(baseboneConstraintType == BaseboneConstraintType2D.GLOBAL_ABSOLUTE && !value) {
			throw new RuntimeException("Cannot set a non-fixed base mode when the chain's constraint type is BaseBoneConstraintType2D.GLOBAL_ABSOLUTE.");
		}

		// Above conditions met? Set the fixedBaseMode
		fixedBaseMode = value;
	}

	/**
	 * Set the maximum number of attempts that will be made to solve this IK chain.
	 * <p>
	 * The FABRIK algorithm may require more than a single pass in order to solve
	 * a given IK chain for an acceptable distance threshold. If we reach this
	 * iteration limit then we stop attempting to solve the IK chain.
	 * <p>
	 * Specifying a maxIterations value of less than 1 will result in an IllegalArgumentException is thrown.
	 *
	 * @param maxIterations The maximum number of attempts that will be made to solve this IK chain.
	 */
	@Override
	public void setMaxIterationAttempts(int maxIterations) {
		// Ensure we have a valid maximum number of iteration attempts
		if(maxIterations < 1) {
			throw new IllegalArgumentException("The maximum number of attempts to solve this IK chain must be at least 1.");
		}

		// All good? Set the new maximum iteration attempts property
		maxIterationAttempts = maxIterations;
	}

	/**
	 * Set the minimum iteration change before we dynamically abort any further attempts to solve this IK chain.
	 * <p>
	 * If the current solve distance changes by less than this amount between solve attempt then we consider the
	 * solve process to have stalled and dynamically abort any further attempts to solve the chain to minimise CPU usage.
	 * <p>
	 * If a minIterationChange value of less than zero is provided then an IllegalArgumentException is thrown.
	 *
	 * @param minIterationChange The minimum change in solve distance from one iteration to the next.
	 */
	@Override
	public void setMinIterationChange(float minIterationChange) {
		// Ensure we have a valid maximum number of iteration attempts
		if(minIterationChange < 0.0f) {
			throw new IllegalArgumentException("The minimum iteration change value must be more than or equal to zero.");
		}

		// All good? Set the new minimum iteration change distance
		this.minIterationChange = minIterationChange;
	}

	/**
	 * Set the distance threshold within which we consider the IK chain to be solved.
	 * <p>
	 * If a solve distance of less than zero is provided then an IllegalArgumentException is thrown.
	 *
	 * @param solveDistance The distance between the end effector of this IK chain and target within which we will accept the solution.
	 */
	@Override
	public void setSolveDistanceThreshold(float solveDistance) {
		// Ensure we have a valid solve distance
		if(solveDistance < 0.0f) {
			throw new IllegalArgumentException("The solve distance threshold must be greater than or equal to zero.");
		}

		// All good? Set the new solve distance threshold
		solveDistanceThreshold = solveDistance;
	}

	/**
	 * Solve the Inverse Kinematics (IK) problem using the FABRIK algorithm in two dimensions.
	 * <p>
	 * The FABRIK algorithm itself can be found in the following paper:
	 * Aristidou, A., & Lasenby, J. (2011). FABRIK: a fast, iterative solver for the inverse kinematics problem. Graphical Models, 73(5), 243-260.
	 * <p>
	 * The end result of running this method is that the IK chain configuration is updated (i.e. joint locations for
	 * each bone in the chain are modified in order to solve the chain for the provided target location).
	 * <p>
	 *
	 * @param targetX The target location x position to solve the IK chain for.
	 * @param targetY The target location y position to solve the IK chain for.
	 * @return float    The distance between the end effector and the given target location after attempting to solve the IK chain.
	 */
	private float solveIK(float targetX, float targetY) {
		// ---------- Step 1 of 2 - Forward pass from end-effector to base -----------

		// Loop over all bones in the chain, from the end effector (numBones-1) back to the base bone (0)		
		for(int loop = this.chain.size - 1; loop >= 0; --loop) {
			// Get this bone
			FabrikBone2D bone = chain.get(loop);

			// Get the length of the bone we're working on
			float boneLength = bone.length();

			// If we are not working on the end effector bone
			if(loop != this.chain.size - 1) {
				FabrikBone2D outerBone = chain.get(loop + 1);

				tmpDir.set(bone.getDirectionUV()).scl(-1f);

				// Constrain the angle between the outer bone and this bone.
				// Note: On the forward pass we constrain to the limits imposed by joint of the outer bone.
				float clockwiseConstraintDegs = outerBone.getJoint().getClockwiseConstraintDegs();
				float antiClockwiseConstraintDegs = outerBone.getJoint().getAnticlockwiseConstraintDegs();

				if(chain.get(loop).getJointConstraintCoordinateSystem() == FabrikJoint2D.ConstraintCoordinateSystem.LOCAL)
					tmpOuterDir.set(outerBone.getDirectionUV()).scl(-1f);
				else
					tmpOuterDir.set(bone.getGlobalConstraintUV()).scl(-1f);

				VectorUtil.getConstrainedUV(tmpDir,
						tmpOuterDir,
						clockwiseConstraintDegs,
						antiClockwiseConstraintDegs,
						tmpConstrainedUV);
			} else {
				// If we are working on the end effector bone ...
				// Snap the end effector's end location to the target
				bone.getEndLocation().set(targetX, targetY);

				// Get the UV between the target / end-location (which are now the same) and the start location of this bone
				tmpDir.set(bone.getDirectionUV()).scl(-1.0f);

				if(loop > 0) {
					// The end-effector bone is NOT the basebone as well

					// Constrain the angle between the this bone and the inner bone
					// Note: On the forward pass we constrain to the limits imposed by the first joint of the inner bone.
					float clockwiseConstraintDegs = bone.getJoint().getClockwiseConstraintDegs();
					float antiClockwiseConstraintDegs = bone.getJoint().getAnticlockwiseConstraintDegs();

					// If this bone is locally constrained...
					if(bone.getJoint().getConstraintCoordinateSystem() == FabrikJoint2D.ConstraintCoordinateSystem.LOCAL) {
						// Get the outer-to-inner unit vector of the bone further in
						tmpInnerDir.set(chain.get(loop - 1).getDirectionUV()).scl(-1.0f);
					} else {
						// End effector bone is globally constrained
						tmpInnerDir.set(bone.getGlobalConstraintUV()).scl(-1.0f);
					}
					// Params: directionUV, baselineUV, clockwise, anticlockwise
					VectorUtil.getConstrainedUV(tmpDir,
							tmpInnerDir,
							clockwiseConstraintDegs,
							antiClockwiseConstraintDegs,
							tmpConstrainedUV);
				} else {
					// There is only one bone in the chain, and the bone is both the basebone and the end-effector bone.

					// Don't constraint (nothing to constraint against) if constraint is in local coordinate system
					if(bone.getJointConstraintCoordinateSystem() == FabrikJoint2D.ConstraintCoordinateSystem.LOCAL) {
						tmpConstrainedUV.set(tmpDir);
					} else {
						// Can constrain if constraining against global coordinate system
						tmpInnerDir.set(bone.getGlobalConstraintUV()).scl(-1.0f);
						VectorUtil.getConstrainedUV(tmpDir,
								tmpInnerDir,
								bone.getClockwiseConstraintDegs(),
								bone.getAnticlockwiseConstraintDegs(),
								tmpConstrainedUV);
					}
				}
			}


			// EITHER
			// At this stage we have a outer-to-inner unit vector for this bone which is within our constraints,
			// so we can set the new inner joint location to be the end joint location of this bone plus the
			// outer-to-inner direction unit vector multiplied by the length of the bone.
			// OR
			// Calculate the new start joint location as the end joint location plus the outer-to-inner direction UV
			// multiplied by the length of the bone.
			// Set the new start joint location for this bone to be new start location...
			bone.getStartLocation().set(bone.getEndLocation()).mulAdd(tmpConstrainedUV, boneLength);

			// EITHER
			// If we are not working on the base bone, then we set the end joint location of
			// the previous bone in the chain (i.e. the bone closer to the base) to be the new
			// start joint location of this bone also.
			// OR
			// ...and set the end joint location of the bone further in to also be at the new start location.
			if(loop > 0) {
				chain.get(loop - 1).setEndLocation(bone.getStartLocation());
			}

		} // End of forward-pass loop over all bones

		// ---------- Step 2 of 2 - Backward pass from base to end effector -----------

		for(int loop = 0; loop < this.chain.size; ++loop) {
			// Get the length of the bone we're working on
			float boneLength = chain.get(loop).length();

			FabrikBone2D thisBone = chain.get(loop);

			// If we are not working on the base bone
			if(loop != 0) {
				FabrikBone2D previousBone = chain.get(loop - 1);

				// Constrain the angle between this bone and the inner bone.
				// Note: On the backward pass we constrain to the limits imposed by the first joint of this bone.
				float clockwiseConstraintDegs = thisBone.getJoint().getClockwiseConstraintDegs();
				float antiClockwiseConstraintDegs = thisBone.getJoint().getAnticlockwiseConstraintDegs();

				if(thisBone.getJointConstraintCoordinateSystem() == FabrikJoint2D.ConstraintCoordinateSystem.LOCAL) {
					VectorUtil.getConstrainedUV(thisBone.getDirectionUV(),
							previousBone.getDirectionUV(),
							clockwiseConstraintDegs,
							antiClockwiseConstraintDegs,
							tmpConstrainedUV);
				} else { // Bone is constrained in global coordinate system
					VectorUtil.getConstrainedUV(thisBone.getDirectionUV(),
							thisBone.getGlobalConstraintUV(),
							clockwiseConstraintDegs,
							antiClockwiseConstraintDegs,
							tmpConstrainedUV);
				}

				// At this stage we have an inner-to-outer unit vector for this bone which is within our constraints,
				// so we can set the new end location to be the start location of this bone plus the constrained
				// inner-to-outer direction unit vector multiplied by the length of this bone.
				thisBone.getEndLocation().set(thisBone.getStartLocation()).mulAdd(tmpConstrainedUV, boneLength);

				// If we are not working on the end bone, then we set the start joint location of
				// the next bone in the chain (i.e. the bone closer to the end effector) to be the
				// new end joint location of this bone also.
				if(loop < this.chain.size - 1) {
					chain.get(loop + 1).setStartLocation(thisBone.getEndLocation());
				}
			} else { // If we ARE working on the base bone...
				// If the base location is fixed then snap the start location of the base bone back to the fixed base
				if(fixedBaseMode) {
					chain.get(0).setStartLocation(baseLocation);
				} else { // If the base location is not fixed...
					// ...then set the new base bone start location to be its the end location minus the
					// bone direction multiplied by the length of the bone (i.e. projected backwards).
					//float boneZeroLength = mChain.get(0).length();
					Vector2 boneZeroUV = chain.get(0).getDirectionUV();
					Vector2 boneZeroEndLocation = chain.get(0).getEndLocation();
					chain.get(0).getStartLocation().set(boneZeroEndLocation).mulAdd(boneZeroUV, -boneLength);
				}

				// If the base bone is unconstrained then process it as usual...
				if(baseboneConstraintType == BaseboneConstraintType2D.NONE) {

					// Calculate the new end location as the start location plus the direction times the length of the bone
					// Set the new end joint location
					chain.get(0).getEndLocation().set(thisBone.getStartLocation()).mulAdd(thisBone.getDirectionUV(), boneLength);

					// Also, set the start location of the next bone to be the end location of this bone
					if(chain.size > 1) {
						chain.get(1).setStartLocation(chain.get(0).getEndLocation());
					}
				} else {
					// ...otherwise we must constrain it to the basebone constraint unit vector

					// Note: The mBaseBoneConstraintUV is either fixed, or it may be dynamically updated from
					// a FabrikStructure2D if this chain is connected to another chain.

					// Get the inner-to-outer direction of this bone

					// Get the constrained direction unit vector between the base bone and the base bone constraint unit vector
					// Note: On the backward pass we constrain to the limits imposed by the first joint of this bone.
					float clockwiseConstraintDegs = thisBone.getJoint().getClockwiseConstraintDegs();
					float antiClockwiseConstraintDegs = thisBone.getJoint().getAnticlockwiseConstraintDegs();

					// LOCAL_ABSOLUTE? (i.e. local-space directional constraint) - then we must constraint about the relative basebone constraint UV...
					if(baseboneConstraintType == BaseboneConstraintType2D.LOCAL_ABSOLUTE) {
						VectorUtil.getConstrainedUV(thisBone.getDirectionUV(),
								baseboneRelativeConstraintUV,
								clockwiseConstraintDegs,
								antiClockwiseConstraintDegs,
								tmpConstrainedUV);

					} else {
						// ...otherwise we're free to use the standard basebone constraint UV.
						VectorUtil.getConstrainedUV(thisBone.getDirectionUV(),
								baseboneConstraintUV,
								clockwiseConstraintDegs,
								antiClockwiseConstraintDegs,
								tmpConstrainedUV);
					}

					// At this stage we have an inner-to-outer unit vector for this bone which is within our constraints,
					// so we can set the new end location to be the start location of this bone plus the constrained
					// inner-to-outer direction unit vector multiplied by the length of the bone.
					// Set the new end joint location for this bone
					chain.get(loop).getEndLocation().set(chain.get(loop).getStartLocation()).mulAdd(tmpConstrainedUV, boneLength);

					// If we are not working on the end bone, then we set the start joint location of
					// the next bone in the chain (i.e. the bone closer to the end effector) to be the
					// new end joint location of this bone.
					if(loop < this.chain.size - 1) {
						chain.get(loop + 1).setStartLocation(chain.get(loop).getEndLocation());
					}

				} // End of basebone constraint enforcement section			

			} // End of base bone handling section

		} // End of backward-pass loop over all bones

		// Update our last target location
		lastTargetLocation.set(targetX, targetY);

		// Finally, get the current effector location and calculate the distance between the
		// current effector location and the target.
		return chain.get(this.chain.size - 1).getEndLocation().dst(targetX, targetY);
	}

	/**
	 * Specify whether we should use the embedded target location when solving the IK chain.
	 *
	 * @param value Whether we should use the embedded target location when solving the IK chain.
	 */
	@Override
	public void setEmbeddedTargetMode(boolean value) {
		useEmbeddedTarget = value;
	}

	/**
	 * Return a concise, human-readable of the IK chain.
	 *
	 * @return String
	 */
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();

		sb.append("----- FabrikChain2D -----\n");
		sb.append("Number of bones: ").append(this.chain.size).append("\n");

		sb.append("Fixed base mode: ");
		if(fixedBaseMode) {
			sb.append("Yes.").append("\n");
		} else {
			sb.append("No.").append("\n");
		}

		sb.append("Base location: ").append(getBaseLocation());

		return sb.toString();
	}

	// ---------- Private Methods ----------

	/**
	 * Clone and return the IK Chain of this FabrikChain2D, that is, the Vector of FabrikBone2D objects.
	 * <p>
	 * This does not clone the entire FabrikChain2D object, only its IK chain.
	 */
	private void cloneChainVector(Array<FabrikBone2D> out) {

		if(out.size != chain.size)
			throw new IllegalArgumentException("Provided out array must contain same number of bone as chain");

		int i = 0;
		for(FabrikBone2D fabrikBone2D : chain)
			out.get(i++).set(fabrikBone2D);
	}

	/***
	 * Calculate the length of this IK chain by adding up the lengths of each bone.
	 * <p>
	 * The resulting chain length is stored in the mChainLength property.
	 * <p>
	 * This method is called each time a bone is added to the chain. In addition, the
	 * length of each bone is recalculated during the process to ensure that our chain
	 * length is accurate. As the typical usage of a FabrikChain2D is to add a number
	 * of bones once (during setup) and then use them, this should not have any
	 * performance implication on the typical execution cycle of a FabrikChain2D object,
	 * as this method will not be called in any method which executes regularly. 
	 */
	@Override
	public void updateChainLength() {
		chainLength = 0.0f;
		for(FabrikBone2D aBone : this.chain) {
			chainLength += aBone.length();
		}
	}


	/**
	 * Update the embedded target for this chain.
	 * <p>
	 * The internal mEmbeddedTarget object is updated with the location of the provided parameter.
	 * If the chain is not in useEmbeddedTarget mode then a RuntimeException is thrown.
	 * Embedded target mode can be enabled by calling setEmbeddedTargetMode(true) on the chain.
	 *
	 * @param newEmbeddedTarget The location of the embedded target.
	 */
	public void updateEmbeddedTarget(Vector2 newEmbeddedTarget) {
		// Using embedded target mode? Overwrite embedded target with provided location
		if(useEmbeddedTarget) {
			embeddedTarget.set(newEmbeddedTarget);
		} else {
			throw new RuntimeException("This chain does not have embedded targets enabled - enable with setEmbeddedTargetMode(true).");
		}
	}

	/**
	 * Update the embedded target for this chain.
	 * <p>
	 * The internal mEmbeddedTarget object is updated with the location of the provided parameter.
	 * If the chain is not in useEmbeddedTarget mode then a RuntimeException is thrown.
	 * Embedded target mode can be enabled by calling setEmbeddedTargetMode(true) on the chain.
	 *
	 * @param x The x location of the embedded target.
	 * @param y The y location of the embedded target.
	 */
	public void updateEmbeddedTarget(float x, float y) {
		// Using embedded target mode? Overwrite embedded target with provided location
		if(useEmbeddedTarget) {
			embeddedTarget.set(x, y);
		} else {
			throw new RuntimeException("This chain does not have embedded targets enabled - enable with setEmbeddedTargetMode(true).");
		}
	}

	/**
	 * Solve this IK chain for the current embedded target location.
	 * <p>
	 * The embedded target location can be updated by calling updateEmbeddedTarget(Vec2f).
	 *
	 * @return The distance between the end effector and the chain's embedded target location for our best solution.
	 */
	@Override
	public float solveForEmbeddedTarget() {
		if(useEmbeddedTarget) {
			return solveForTarget(embeddedTarget);
		} else {
			throw new RuntimeException("This chain does not have embedded targets enabled - enable with setEmbeddedTargetMode(true).");
		}
	}

	/**
	 * Solve the IK chain for this target to the best of our ability.
	 * <p>
	 * We will iteratively attempt up to solve the chain up to a maximum of mMaxIterationAttempts.
	 * <p>
	 * This method may return early if any of the following conditions are met:
	 * <ul>
	 * <li>We've already solved for this target location,</li>
	 * <li>We successfully solve for distance, or</li>
	 * <li>We grind to a halt (i.e. low iteration change compared to previous solution).</li>
	 * </ul>
	 * <p>
	 * This method simply constructs a Vec2f from the provided arguments and calls the {@link Vector2} version of the solveForTarget method.
	 * By making this available, the user does not need to use our custom {@link Vector2} class in their application.
	 *
	 * @param newTarget The target location to solve this IK chain for.
	 * @return The distance between the end effector and the target for our best solution
	 * @see #maxIterationAttempts
	 * @see #solveDistanceThreshold
	 * @see #minIterationChange
	 */
	public float solveForTarget(Vector2 newTarget) {
		return solveForTarget(newTarget.x, newTarget.y);
	}

	private void ensureChainSize(Array<FabrikBone2D> array) {
		while(array.size < this.chain.size)
			array.add(new FabrikBone2D());

		while(array.size > chain.size)
			array.removeIndex(array.size - 1);
	}

	/**
	 * Solve the IK chain for this target to the best of our ability.
	 * <p>
	 * We will iteratively attempt up to solve the chain up to a maximum of mMaxIterationAttempts.
	 * <p>
	 * This method may return early if any of the following conditions are met:
	 * <ul>
	 * <li>We've already solved for this target location,</li>
	 * <li>We successfully solve for distance, or</li>
	 * <li>We grind to a halt (i.e. low iteration change compared to previous solution).</li>
	 * </ul>
	 * <p>
	 * The solution may NOT change we cannot improve upon the calculated distance between the new target location and the existing solution.
	 *
	 * @param targetX (float) The x location of the target.
	 * @param targetY (float) The y location of the target.
	 * @return The distance between the end effector and the target for our best solution
	 * @see #maxIterationAttempts
	 * @see #solveDistanceThreshold
	 * @see #minIterationChange
	 */
	public float solveForTarget(float targetX, float targetY) {
		// Same target as before? Abort immediately and save ourselves some cycles
		if(lastTargetLocation.epsilonEquals(targetX, targetY, 0.001f)
				&& lastBaseLocation.epsilonEquals(baseLocation, 0.001f)) {
			return currentSolveDistance;
		}

		ensureChainSize(tmpBestSolution);
		ensureChainSize(tmpStartSolution);

		// Keep starting solutions and distance
		float startingDistance;

		cloneChainVector(tmpStartSolution);

		// If the base location of a chain hasn't moved then we may opt to keep the current solution if our 
		// best new solution is worse...
		if(lastBaseLocation.epsilonEquals(baseLocation, 0.001f)) {
			startingDistance = chain.get(chain.size - 1).getEndLocation().dst(targetX, targetY);
		} else { // Base has changed? Then we have little choice but to recalc the solution and take that new solution.
			startingDistance = Float.MAX_VALUE;
		}

		// We'll keep track of our best solve distance, starting it at a huge value which will be beaten on first attempt			
		float bestSolveDistance = Float.MAX_VALUE;
		float lastPassSolveDistance = Float.MAX_VALUE;

		// Allow up to our iteration limit attempts at solving the chain
		float solveDistance;
		for(int loop = 0; loop < maxIterationAttempts; ++loop) {
			// Solve the chain for this target
			solveDistance = solveIK(targetX, targetY);

			// Did we solve it for distance? If so, update our best distance and best solution, and also
			// update our last pass solve distance. Note: We will ALWAYS beat our last solve distance on the first run. 
			if(solveDistance < bestSolveDistance) {
				bestSolveDistance = solveDistance;
				cloneChainVector(tmpBestSolution);

				// Did we solve for distance? Great! Break out of the loop.
				if(solveDistance <= solveDistanceThreshold) {
					break;
				}
			} else { // Did not solve to our satisfaction? Okay...

				// Did we grind to a halt? If so then it's time to break out of the loop.
				if(Math.abs(solveDistance - lastPassSolveDistance) < minIterationChange) {
					break;
				}
			}

			// Update the last pass solve distance
			lastPassSolveDistance = solveDistance;
		}

		// Did we get a solution that's better than the starting solution's to the new target location?
		if(bestSolveDistance < startingDistance) {
			// If so, set the newly found solve distance and solution as the best found.
			currentSolveDistance = bestSolveDistance;
			for(int i = 0; i < chain.size; i++)
				chain.get(i).set(tmpBestSolution.get(i));
		} else { // Did we make things worse? Then we keep our starting distance and solution!
			currentSolveDistance = startingDistance;
			for(int i = 0; i < chain.size; i++)
				chain.get(i).set(tmpStartSolution.get(i));
		}

		// Update our last base and target locations so we know whether we need to solve for this start/end configuration next time
		lastBaseLocation.set(baseLocation);
		lastTargetLocation.set(targetX, targetY);

		// Finally, return the solve distance we ended up with
		return currentSolveDistance;
	}

	/**
	 * Return the relative constraint UV about which this bone connects to a bone in another chain.
	 *
	 * @return The relative constraint UV about which this bone connects to a bone in another chain.
	 */
	@Override
	public Vector2 getBaseboneRelativeConstraintUV() {
		return baseboneRelativeConstraintUV;
	}

	/**
	 * Set the constraint UV about which this bone connects to a bone in another chain.
	 *
	 * @param constraintUV The basebone relative constraint unit vector to set.
	 */
	public void setBaseboneRelativeConstraintUV(Vector2 constraintUV) {
		baseboneRelativeConstraintUV.set(constraintUV);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public int getMaxIterationAttempts() {
		return this.maxIterationAttempts;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public float getMinIterationChange() {
		return this.minIterationChange;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public float getSolveDistanceThreshold() {
		return this.solveDistanceThreshold;
	}

	public void setLastBaseLocation(float x, float y) {
		this.lastBaseLocation.set(x, y);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((baseLocation == null) ? 0 : baseLocation.hashCode());
		result = prime * result + ((baseboneConstraintType == null) ? 0 : baseboneConstraintType.hashCode());
		result = prime * result + ((baseboneConstraintUV == null) ? 0 : baseboneConstraintUV.hashCode());
		result = prime * result + ((baseboneRelativeConstraintUV == null) ? 0 : baseboneRelativeConstraintUV.hashCode());
		result = prime * result + ((boneConnectionPoint == null) ? 0 : boneConnectionPoint.hashCode());
		result = prime * result + ((chain == null) ? 0 : chain.hashCode());
		result = prime * result + Float.floatToIntBits(chainLength);
		result = prime * result + connectedBoneNumber;
		result = prime * result + connectedChainNumber;
		result = prime * result + Float.floatToIntBits(currentSolveDistance);
		result = prime * result + ((embeddedTarget == null) ? 0 : embeddedTarget.hashCode());
		result = prime * result + (fixedBaseMode ? 1231 : 1237);
		result = prime * result + ((lastBaseLocation == null) ? 0 : lastBaseLocation.hashCode());
		result = prime * result + ((lastTargetLocation == null) ? 0 : lastTargetLocation.hashCode());
		result = prime * result + maxIterationAttempts;
		result = prime * result + Float.floatToIntBits(minIterationChange);
		result = prime * result + Float.floatToIntBits(solveDistanceThreshold);
		result = prime * result + (useEmbeddedTarget ? 1231 : 1237);
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
		FabrikChain2D other = (FabrikChain2D)obj;
		if(!baseLocation.equals(other.baseLocation)) {
			return false;
		}
		if(baseboneConstraintType != other.baseboneConstraintType) {
			return false;
		}
		if(!baseboneConstraintUV.equals(other.baseboneConstraintUV)) {
			return false;
		}
		if(!baseboneRelativeConstraintUV.equals(other.baseboneRelativeConstraintUV)) {
			return false;
		}
		if(boneConnectionPoint != other.boneConnectionPoint) {
			return false;
		}
		if(chain == null) {
			if(other.chain != null) {
				return false;
			}
		} else if(!chain.equals(other.chain)) {
			return false;
		}
		if(Float.floatToIntBits(chainLength) != Float.floatToIntBits(other.chainLength)) {
			return false;
		}
		if(connectedBoneNumber != other.connectedBoneNumber) {
			return false;
		}
		if(connectedChainNumber != other.connectedChainNumber) {
			return false;
		}
		if(Float.floatToIntBits(currentSolveDistance) != Float.floatToIntBits(other.currentSolveDistance)) {
			return false;
		}
		if(!embeddedTarget.equals(other.embeddedTarget)) {
			return false;
		}
		if(fixedBaseMode != other.fixedBaseMode) {
			return false;
		}
		if(!lastBaseLocation.equals(other.lastBaseLocation)) {
			return false;
		}
		if(!lastTargetLocation.equals(other.lastTargetLocation)) {
			return false;
		}
		if(maxIterationAttempts != other.maxIterationAttempts) {
			return false;
		}
		if(Float.floatToIntBits(minIterationChange) != Float.floatToIntBits(other.minIterationChange)) {
			return false;
		}
		if(Float.floatToIntBits(solveDistanceThreshold) != Float.floatToIntBits(other.solveDistanceThreshold)) {
			return false;
		}
		return useEmbeddedTarget == other.useEmbeddedTarget;
	}
} // End of FabrikChain2D class
