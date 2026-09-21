/*
 * ControllerGainPair.h
 *
 * Apply scalar or move/standstill controller gain updates consistently.
 */

#ifndef SRC_CLOSEDLOOP_CONTROLLERGAINPAIR_H_
#define SRC_CLOSEDLOOP_CONTROLLERGAINPAIR_H_

namespace ControllerGainPair
{
	inline void Apply(bool primarySeen, bool secondarySeen, float newPrimary, float newSecondary,
		float& primary, float& secondary, bool& pairEnabled) noexcept
	{
		if (!primarySeen)
		{
			return;
		}

		primary = newPrimary;
		if (secondarySeen)
		{
			secondary = newSecondary;
			pairEnabled = true;
		}
		else
		{
			secondary = newPrimary;
			pairEnabled = false;
		}
	}
}

#endif
