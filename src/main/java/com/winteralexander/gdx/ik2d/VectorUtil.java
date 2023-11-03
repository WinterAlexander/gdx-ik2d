package com.winteralexander.gdx.ik2d;

import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;

/**
 * Utility for vector operations
 * <p>
 * Created on 2023-11-01.
 *
 * @author Alexander Winter
 */
public class VectorUtil {

	public static float getSignedAngleDegsTo(Vector2 first, Vector2 second)
	{
		if(first.len2() == 0.0f || second.len2() == 0.0f)
			return 0.0f;

		// Normalise the vectors that we're going to use
		if(Math.abs(first.len2() - 1.0) > MathUtils.FLOAT_ROUNDING_ERROR)
			throw new IllegalArgumentException("first argument to getSignedAngleDegsTo is not unit length");
		if(Math.abs(second.len2() - 1.0) > MathUtils.FLOAT_ROUNDING_ERROR)
			throw new IllegalArgumentException("second argument to getSignedAngleDegsTo is not unit length");

		// Calculate the unsigned angle between the vectors as the arc-cosine of their dot product
		float unsignedAngleDegs = (float)Math.acos( first.dot(second) ) * MathUtils.radiansToDegrees;

		// Calculate and return the signed angle between the two vectors using the zcross method
		return unsignedAngleDegs * Math.signum(first.x * second.y - second.x * first.y);
	}

	public static void getConstrainedUV(Vector2 directionUV,
	                                    Vector2 baselineUV,
	                                    float clockwiseConstraintDegs,
	                                    float antiClockwiseConstraintDegs,
	                                    Vector2 out)
	{
		// Get the signed angle from the baseline UV to the direction UV.
		// Note: In our signed angle ranges:
		//       0...180 degrees represents anti-clockwise rotation, and
		//       0..-180 degrees represents clockwise rotation
		float signedAngleDegs = getSignedAngleDegsTo(baselineUV, directionUV);

		// If we've exceeded the anti-clockwise (positive) constraint angle...
		if (signedAngleDegs > antiClockwiseConstraintDegs)
		{
			// ...then our constrained unit vector is the baseline rotated by the anti-clockwise constraint angle.
			// Note: We could do this by calculating a correction angle to apply to the directionUV, but it's simpler to work from the baseline.
			out.set(baselineUV).rotateDeg(antiClockwiseConstraintDegs);
		}

		// If we've exceeded the clockwise (negative) constraint angle...
		if (signedAngleDegs < -clockwiseConstraintDegs)
		{
			// ...then our constrained unit vector is the baseline rotated by the clockwise constraint angle.
			// Note: Again, we could do this by calculating a correction angle to apply to the directionUV, but it's simpler to work from the baseline.
			out.set(baselineUV).rotateDeg(-clockwiseConstraintDegs);
		}

		// If we have not exceeded any constraint then we simply return the original direction unit vector
		out.set(directionUV);
	}
}
