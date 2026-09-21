#include "../src/ClosedLoop/MotionTransition.h"
#include "../src/ClosedLoop/SplitIntegral.h"

#include <cassert>
#include <cmath>

namespace
{
	bool Near(float a, float b, float tolerance = 1.0e-5f)
	{
		return std::fabs(a - b) <= tolerance;
	}

	void TestMotionBlend()
	{
		constexpr uint32_t ticks = 100;
		constexpr unsigned int updates = 8;
		constexpr float acceleration = 2.0e-8f;

		float previous = 0.0f;
		assert(MotionTransition::Calculate(0.0f, 0.0f, ticks, updates) == 0.0f);
		for (unsigned int update = 1; update <= updates; ++update)
		{
			const float speed = acceleration * (float)(ticks * update);
			const float blend = MotionTransition::Calculate(speed, acceleration, ticks, updates);
			assert(Near(blend, MotionTransition::SmoothStep((float)update/(float)updates)));
			assert(blend >= previous);
			previous = blend;
		}
		assert(previous == 1.0f);
		assert(MotionTransition::Calculate(1.0e-4f, 0.0f, ticks, updates) == 1.0f);

		previous = 1.0f;
		for (unsigned int remaining = updates; remaining != 0; --remaining)
		{
			const float speed = acceleration * (float)(ticks * (remaining - 1));
			const float blend = MotionTransition::Calculate(speed, -acceleration, ticks, updates);
			assert(blend <= previous);
			previous = blend;
		}
		assert(previous == 0.0f);
	}

	void TestFrictionFeedforward()
	{
		FrictionFeedforward friction;
		assert(friction.Update(0.0f, 72.0f, 0.0f, 8) == 0.0f);
		assert(Near(friction.Update(1.0f, 72.0f, 0.5f, 8), 36.0f));
		assert(Near(friction.Update(1.0f, 72.0f, 1.0f, 8), 72.0f));
		assert(friction.Update(0.0f, 72.0f, 0.0f, 8) == 0.0f);
		assert(Near(friction.Update(-1.0f, 72.0f, 1.0f, 8), -72.0f));

		friction.Reset();
		assert(Near(friction.Update(1.0f, 72.0f, 1.0f, 8), 72.0f));
		assert(friction.Update(-1.0f, 72.0f, 1.0f, 8) == 0.0f);
		float lastMagnitude = 0.0f;
		for (unsigned int update = 1; update <= 8; ++update)
		{
			const float value = friction.Update(-1.0f, 72.0f, 1.0f, 8);
			assert(value <= 0.0f);
			assert(std::fabs(value) >= lastMagnitude);
			lastMagnitude = std::fabs(value);
		}
		assert(Near(lastMagnitude, 72.0f));
	}

	void TestSplitIntegral()
	{
		float integralError = 0.0f;
		float term = SplitIntegral::Update(integralError, 5.0f, 2.0f, 0.1f, 0.0f, 80.0f);
		assert(Near(term, 1.0f));
		const float storedAtStandstill = integralError;

		term = SplitIntegral::Update(integralError, 0.0f, 100.0f, 1.0f, 0.0f, 80.0f);
		assert(term == 0.0f);
		assert(integralError == storedAtStandstill);

		term = SplitIntegral::Update(integralError, 2.5f, 0.0f, 0.1f, 0.0f, 80.0f);
		assert(Near(term, 0.5f));
		term = SplitIntegral::Update(integralError, 5.0f, 0.0f, 0.1f, 0.0f, 80.0f);
		assert(Near(term, 1.0f));

		integralError = 0.0f;
		term = SplitIntegral::Update(integralError, 5.0f, 1.0f, 1.0f, 250.0f, 80.0f);
		assert(Near(term, 5.0f));
		term = SplitIntegral::Update(integralError, 5.0f, 1.0f, 1.0f, 250.0f, 80.0f);
		assert(Near(term, 5.0f));
		term = SplitIntegral::Update(integralError, 5.0f, -1.0f, 1.0f, 260.0f, 80.0f);
		assert(Near(term, 0.0f));

		integralError = 1000.0f;
		term = SplitIntegral::Update(integralError, 5.0f, 0.0f, 1.0f, 0.0f, 80.0f);
		assert(Near(term, 80.0f));
		assert(Near(integralError, 16.0f));
	}
}

int main()
{
	TestMotionBlend();
	TestFrictionFeedforward();
	TestSplitIntegral();
	return 0;
}
