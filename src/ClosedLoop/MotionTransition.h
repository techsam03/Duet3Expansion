/*
 * MotionTransition.h
 *
 * Small stateful transitions used by the closed-loop controller.
 */

#ifndef SRC_CLOSEDLOOP_MOTIONTRANSITION_H_
#define SRC_CLOSEDLOOP_MOTIONTRANSITION_H_

#include <cmath>
#include <cstdint>

namespace MotionTransition
{
	constexpr float SmoothStep(float x) noexcept
	{
		const float limited = (x < 0.0f) ? 0.0f : ((x > 1.0f) ? 1.0f : x);
		return limited * limited * (3.0f - 2.0f * limited);
	}

}

// Blend between standstill (0) and commanded movement (1). The state depends
// only on the commanded trajectory, never on encoder-derived velocity.
class MoveStandstillTransition
{
public:
	void Reset() noexcept { transitionPosition = 0; }

	float Update(float targetSpeed, float targetAcceleration, unsigned int transitionUpdates) noexcept
	{
		if (transitionUpdates == 0)
		{
			transitionPosition = 0;
			return (targetSpeed != 0.0f || targetAcceleration != 0.0f) ? 1.0f : 0.0f;
		}

		const bool moving = targetSpeed != 0.0f || targetAcceleration != 0.0f;
		if (moving)
		{
			if (transitionPosition < transitionUpdates)
			{
				++transitionPosition;
			}
		}
		else if (transitionPosition != 0)
		{
			--transitionPosition;
		}

		return MotionTransition::SmoothStep((float)transitionPosition/(float)transitionUpdates);
	}

private:
	unsigned int transitionPosition = 0;
};

class FrictionFeedforward
{
public:
	void Reset() noexcept
	{
		signedTransitionPosition = 0;
	}

	float Update(float targetSpeed, float magnitude, unsigned int transitionUpdates) noexcept
	{
		if (magnitude == 0.0f)
		{
			Reset();
			return 0.0f;
		}

		const int requestedDirection = (targetSpeed > 0.0f) ? 1 : ((targetSpeed < 0.0f) ? -1 : 0);
		if (transitionUpdates == 0)
		{
			signedTransitionPosition = 0;
			return (float)requestedDirection * magnitude;
		}

		const int targetPosition = requestedDirection * (int)transitionUpdates;
		if (signedTransitionPosition < targetPosition)
		{
			++signedTransitionPosition;
		}
		else if (signedTransitionPosition > targetPosition)
		{
			--signedTransitionPosition;
		}

		if (signedTransitionPosition == 0)
		{
			return 0.0f;
		}

		const float fraction = MotionTransition::SmoothStep(
			(float)std::abs(signedTransitionPosition)/(float)transitionUpdates);
		return ((signedTransitionPosition > 0) ? magnitude : -magnitude) * fraction;
	}

private:
	int signedTransitionPosition = 0;
};

#endif
