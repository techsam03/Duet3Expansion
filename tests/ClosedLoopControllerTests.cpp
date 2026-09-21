#include "../src/ClosedLoop/ControllerGainPair.h"
#include "../src/ClosedLoop/MotionTransition.h"
#include "../src/ClosedLoop/SplitIntegral.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{
	constexpr unsigned int MoveStandstillTransitionUpdates = 24;
	constexpr unsigned int FrictionTransitionUpdates = 8;

	bool Near(float a, float b, float tolerance = 1.0e-5f)
	{
		return std::fabs(a - b) <= tolerance;
	}

	void TestMoveStandstillTransition()
	{
		MoveStandstillTransition transition;
		assert(transition.Update(0.0f, 0.0f, MoveStandstillTransitionUpdates) == 0.0f);

		float previous = 0.0f;
		for (unsigned int update = 1; update <= MoveStandstillTransitionUpdates; ++update)
		{
			const float blend = transition.Update(0.01f * (float)update, 1.0e-5f, MoveStandstillTransitionUpdates);
			assert(Near(blend, MotionTransition::SmoothStep((float)update/(float)MoveStandstillTransitionUpdates)));
			assert(blend >= previous);
			previous = blend;
		}
		assert(previous == 1.0f);

		// Acceleration, cruise, non-zero junction deceleration, small non-zero
		// velocity and acceleration away from a junction all remain MOVE.
		assert(transition.Update(0.05f, 1.0e-5f, MoveStandstillTransitionUpdates) == 1.0f);
		assert(transition.Update(0.05f, 0.0f, MoveStandstillTransitionUpdates) == 1.0f);
		assert(transition.Update(0.01f, -1.0e-5f, MoveStandstillTransitionUpdates) == 1.0f);
		assert(transition.Update(0.01f, 0.0f, MoveStandstillTransitionUpdates) == 1.0f);
		assert(transition.Update(0.05f, 1.0e-5f, MoveStandstillTransitionUpdates) == 1.0f);
		assert(transition.Update(0.0f, -1.0e-5f, MoveStandstillTransitionUpdates) == 1.0f);

		previous = 1.0f;
		for (unsigned int remaining = MoveStandstillTransitionUpdates - 1; ; --remaining)
		{
			const float blend = transition.Update(0.0f, 0.0f, MoveStandstillTransitionUpdates);
			assert(Near(blend, MotionTransition::SmoothStep((float)remaining/(float)MoveStandstillTransitionUpdates)));
			assert(blend <= previous);
			previous = blend;
			if (remaining == 0) { break; }
		}
		assert(previous == 0.0f);

		// No encoder measurement is accepted by the state machine, so encoder
		// noise cannot alter the standstill state.
		assert(transition.Update(0.0f, 0.0f, MoveStandstillTransitionUpdates) == 0.0f);
	}

	void TestFrictionFeedforward()
	{
		FrictionFeedforward friction;
		assert(friction.Update(0.0f, 72.0f, FrictionTransitionUpdates) == 0.0f);
		float previous = 0.0f;
		for (unsigned int update = 1; update <= FrictionTransitionUpdates; ++update)
		{
			const float value = friction.Update(0.01f, 72.0f, FrictionTransitionUpdates);
			assert(Near(value, 72.0f * MotionTransition::SmoothStep((float)update/(float)FrictionTransitionUpdates)));
			assert(value >= previous);
			previous = value;
		}
		assert(Near(previous, 72.0f));

		// A 50 -> 10 mm/s junction remains non-zero commanded motion.
		assert(Near(friction.Update(0.05f, 72.0f, FrictionTransitionUpdates), 72.0f));
		assert(Near(friction.Update(0.01f, 72.0f, FrictionTransitionUpdates), 72.0f));

		for (unsigned int remaining = FrictionTransitionUpdates - 1; ; --remaining)
		{
			const float value = friction.Update(0.0f, 72.0f, FrictionTransitionUpdates);
			assert(Near(value, 72.0f * MotionTransition::SmoothStep((float)remaining/(float)FrictionTransitionUpdates)));
			if (remaining == 0) { break; }
		}
		assert(friction.Update(0.0f, 72.0f, FrictionTransitionUpdates) == 0.0f);

		// An abnormal direct sign reversal must cross zero before becoming negative.
		friction.Reset();
		for (unsigned int i = 0; i < FrictionTransitionUpdates; ++i)
		{
			(void)friction.Update(1.0f, 72.0f, FrictionTransitionUpdates);
		}
		for (unsigned int i = 0; i < FrictionTransitionUpdates; ++i)
		{
			assert(friction.Update(-1.0f, 72.0f, FrictionTransitionUpdates) >= 0.0f);
		}
		assert(friction.Update(-1.0f, 72.0f, FrictionTransitionUpdates) < 0.0f);

		assert(friction.Update(1.0f, 0.0f, FrictionTransitionUpdates) == 0.0f);
		assert(friction.Update(-1.0f, 0.0f, FrictionTransitionUpdates) == 0.0f);
	}

	void TestSplitIntegral()
	{
		float integralError = 0.0f;
		float term = SplitIntegral::Update(integralError, 5.0f, 2.0f, 0.1f, 0.0f, 80.0f);
		assert(Near(term, 1.0f));
		const float storedAtStandstill = integralError;

		// I0:5 output follows the blended effective gain to zero over eight
		// movement updates instead of freezing the standstill output.
		MoveStandstillTransition transition;
		float previous = term;
		for (unsigned int update = 1; update <= MoveStandstillTransitionUpdates; ++update)
		{
			const float blend = transition.Update(1.0f, 0.0f, MoveStandstillTransitionUpdates);
			const float gain = 5.0f * (1.0f - blend);
			term = SplitIntegral::Update(integralError, gain, 0.0f, 0.1f, 0.0f, 80.0f);
			assert(term <= previous);
			previous = term;
		}
		assert(term == 0.0f);

		// A zero effective gain produces no output and adds no hidden error.
		term = SplitIntegral::Update(integralError, 0.0f, 100.0f, 1.0f, 0.0f, 80.0f);
		assert(term == 0.0f);
		assert(integralError == storedAtStandstill);
		term = SplitIntegral::Update(integralError, 1.0e-9f, 100.0f, 1.0f, 0.0f, 80.0f);
		assert(term == 0.0f);
		assert(std::isfinite(integralError));

		// Block integration into positive saturation and permit unwind.
		integralError = 0.0f;
		term = SplitIntegral::Update(integralError, 5.0f, 1.0f, 1.0f, 255.0f, 80.0f);
		assert(term == 0.0f);
		term = SplitIntegral::Update(integralError, 5.0f, -1.0f, 1.0f, 260.0f, 80.0f);
		assert(Near(term, -5.0f));

		// The existing I contribution limit maps safely back to shared state.
		integralError = 1000.0f;
		term = SplitIntegral::Update(integralError, 5.0f, 0.0f, 1.0f, 0.0f, 80.0f);
		assert(Near(term, 80.0f));
		assert(Near(integralError, 16.0f));
		assert(std::isfinite(term) && std::isfinite(integralError));
	}

	void CheckGainText(float primary, float secondary, bool paired, const char* expected)
	{
		char text[40];
		if (paired)
		{
			std::snprintf(text, sizeof(text), "%.3f:%.3f", (double)primary, (double)secondary);
		}
		else
		{
			std::snprintf(text, sizeof(text), "%.3f", (double)primary);
		}
		assert(std::strcmp(text, expected) == 0);
	}

	void TestGainPairSwitching()
	{
		float primary = 0.0f, secondary = 0.0f;
		bool paired = false;

		ControllerGainPair::Apply(true, true, 0.0f, 5.0f, primary, secondary, paired);
		CheckGainText(primary, secondary, paired, "0.000:5.000");
		ControllerGainPair::Apply(true, false, 5.0f, 123.0f, primary, secondary, paired);
		CheckGainText(primary, secondary, paired, "5.000");
		assert(secondary == primary);
		ControllerGainPair::Apply(true, true, 2.0f, 7.0f, primary, secondary, paired);
		CheckGainText(primary, secondary, paired, "2.000:7.000");
		ControllerGainPair::Apply(true, false, 3.0f, 123.0f, primary, secondary, paired);
		CheckGainText(primary, secondary, paired, "3.000");

		ControllerGainPair::Apply(true, true, 0.30f, 0.0f, primary, secondary, paired);
		CheckGainText(primary, secondary, paired, "0.300:0.000");
		ControllerGainPair::Apply(true, false, 0.30f, 1.0f, primary, secondary, paired);
		CheckGainText(primary, secondary, paired, "0.300");
		assert(secondary == primary);
		ControllerGainPair::Apply(true, true, 0.40f, 0.05f, primary, secondary, paired);
		CheckGainText(primary, secondary, paired, "0.400:0.050");
		ControllerGainPair::Apply(true, false, 0.25f, 1.0f, primary, secondary, paired);
		CheckGainText(primary, secondary, paired, "0.250");
	}
}

int main()
{
	TestMoveStandstillTransition();
	TestFrictionFeedforward();
	TestSplitIntegral();
	TestGainPairSwitching();
	return 0;
}
