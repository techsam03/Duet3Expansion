/*
 * MotionTransition.h
 *
 * Helpers shared by the closed-loop I, D and friction terms.
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

	// speed is in full steps/tick and acceleration is in full steps/tick^2.
	inline float Calculate(float speed, float acceleration, uint32_t ticksSinceLastCall, unsigned int transitionUpdates) noexcept
	{
		if (speed == 0.0f && acceleration == 0.0f)
		{
			return 0.0f;
		}
		if (acceleration == 0.0f)
		{
			return 1.0f;
		}

		const float transitionTicks = (float)transitionUpdates * (float)ticksSinceLastCall;
		const float velocityChange = fabsf(acceleration) * transitionTicks;
		return (velocityChange > 0.0f) ? SmoothStep(fabsf(speed)/velocityChange) : 1.0f;
	}
}

class FrictionFeedforward
{
public:
	void Reset() noexcept
	{
		lastDirection = reversalDirection = 0;
		reversalUpdate = 0;
	}

	float Update(float speed, float magnitude, float motionBlend, unsigned int transitionUpdates) noexcept
	{
		const int8_t direction = (speed > 0.0f) ? 1 : ((speed < 0.0f) ? -1 : 0);
		if (direction == 0 || magnitude == 0.0f)
		{
			Reset();
			return 0.0f;
		}

		// A normal planned reversal passes through zero. Protect against an abnormal
		// direct sign change by reacquiring the new direction over the same horizon.
		if (lastDirection != 0 && direction != lastDirection)
		{
			lastDirection = direction;
			reversalDirection = direction;
			reversalUpdate = 0;
			return 0.0f;
		}

		lastDirection = direction;
		float reversalBlend = 1.0f;
		if (reversalDirection != 0)
		{
			if (reversalUpdate < transitionUpdates)
			{
				++reversalUpdate;
			}
			reversalBlend = MotionTransition::SmoothStep((float)reversalUpdate/(float)transitionUpdates);
			if (reversalUpdate == transitionUpdates)
			{
				reversalDirection = 0;
			}
		}
		return (float)direction * magnitude * motionBlend * reversalBlend;
	}

private:
	int8_t lastDirection = 0;
	int8_t reversalDirection = 0;
	unsigned int reversalUpdate = 0;
};

#endif
