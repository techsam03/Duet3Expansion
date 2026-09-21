/*
 * SplitIntegral.h
 */

#ifndef SRC_CLOSEDLOOP_SPLITINTEGRAL_H_
#define SRC_CLOSEDLOOP_SPLITINTEGRAL_H_

namespace SplitIntegral
{
	inline float Update(float& integralError, float gain, float positionError, float timeDelta,
		float baseWithoutI, float integralLimit) noexcept
	{
		constexpr float MinimumEffectiveGain = 1.0e-7f;
		if (gain <= MinimumEffectiveGain)
		{
			// Preserve the shared state, but do not accumulate error invisibly.
			return 0.0f;
		}

		const float rawPrevious = gain * integralError;
		const float previous = (rawPrevious < -integralLimit) ? -integralLimit
			: ((rawPrevious > integralLimit) ? integralLimit : rawPrevious);
		if (previous != rawPrevious)
		{
			integralError = previous/gain;
		}

		const float candidateError = integralError + positionError * timeDelta;
		const float rawCandidate = gain * candidateError;
		const float candidate = (rawCandidate < -integralLimit) ? -integralLimit
			: ((rawCandidate > integralLimit) ? integralLimit : rawCandidate);
		const float outputChange = candidate - previous;
		const float candidateTotal = baseWithoutI + candidate;
		if ((candidateTotal >= -256.0f && candidateTotal <= 256.0f)
			|| (candidateTotal > 256.0f && outputChange < 0.0f)
			|| (candidateTotal < -256.0f && outputChange > 0.0f))
		{
			// The normal path needs no division. Divide only when the requested I
			// contribution had to be clamped back to the existing limit.
			integralError = (candidate == rawCandidate) ? candidateError : candidate/gain;
			return candidate;
		}
		return previous;
	}
}

#endif
