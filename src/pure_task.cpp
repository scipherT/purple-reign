#include <pure_task.h>

// #define LOG_MISSED_TICKS

using namespace PurpleReign;

PurpleReign::Task::Task()
{
	m_function = nullptr;
	m_periodInMicros = 0;
	m_nextTickInMicros = 0;

}

PurpleReign::Task::Task(void (*function)(), unsigned long periodInMicros)
{
	m_function = function;
	m_periodInMicros = periodInMicros;
	m_nextTickInMicros = 0;
}

void PurpleReign::Task::init()
{
	return;
}

void PurpleReign::Task::setFunction(void (*function)())
{
	m_function = function;
}

void PurpleReign::Task::setPeriod(unsigned long periodInMicros)
{
	m_periodInMicros = periodInMicros;
}

void PurpleReign::Task::schedule()
{
	unsigned long timeNowInMicros = micros();
	// Only do stuff if tick timer is due
	if (timeNowInMicros > m_nextTickInMicros)
	{
		// Handle missed/not_missed ticks
		if (timeNowInMicros > m_nextTickInMicros + m_periodInMicros)
		{ //missed one or more ticks!
#ifdef LOG_MISSED_TICKS
			logMissedTicks(timeNowInMicros, (timeNowInMicros - nextTickInMicros) / periodInMicros);
#endif
			m_nextTickInMicros = timeNowInMicros - (timeNowInMicros % m_periodInMicros) + m_periodInMicros; // FF to closest future next tick (= smallest future integer multiple of a period)
																											// TODO (OPTIMIZE): This code should probably not happen often, if at all. Still... Use tick intervals with size 2^n.
																											// This means "Fast Forward" can be done easier, e.g. by setting the current time "tick fraction" bits to zero and do +1 on "tick integer" .
																											// E.g.;
																											// * tick_size is 512 us (= 2^9 us)
																											// * Assume now that first tick = 0 (= no offset) and [current_time > (next_tick + tick_size)] i.e.; missed one or more ticks
																											// * [CODE] if (current_time > next_tick + tick_size )  { next_tick = (time && FFFFFE00) + tick_size;}
																											// * The above is correct *iff* the missed tick does not happen so that current time occurs exactly on an "integer" tick time,
																											// i.e. the "fraction" tick is zero. In that rare case one tick too much will be skipped. Otoh, skipping one tick too much might be OK once in a while.
		}
		else
		{ // Didn't miss any tick(s), just advance to next tick
			m_nextTickInMicros += m_periodInMicros;
		}
		m_function(); // call the member function (implicitly dereferencing the member function pointer)
	}
}
