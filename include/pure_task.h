#ifndef PURE_TASK_H
#define PURE_TASK_H

#include <Arduino.h>

namespace PurpleReign
{

	class Task
	{
	private:
		unsigned long m_periodInMicros;
		unsigned long m_nextTickInMicros;
		void (*m_function)();

	public:
		// static void updateTime();
		Task();
		Task(void (*function)(), unsigned long periodInMicros);
		void init();
		void setFunction(void (*function)());
		void setPeriod(unsigned long periodInMicros);
		void schedule(); // Schedules based on actual time of method invocation. Calling schedule() will recalculate the current time for each invocation.
	};

}

#endif /* PURE_TASK_H */