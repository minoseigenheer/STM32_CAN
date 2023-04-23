/*
 * scheduler.hpp
 *
 *  Created on: 11 feb. 2023
 *      Author: minoseigenheer
 */

#ifndef SCHEDULER_HPP_
#define SCHEDULER_HPP_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"


using tSchedulerTime = uint32_t;
constexpr tSchedulerTime SchedulerDisabled = -1; // 0xffffffffUL or 0xffffffffffffffffULL
//------------------------------------------------------------------------------
// SyncScheduler is meant to schedule periodic function calls
// for example sending periodic CAN messages

class tSyncScheduler {
  protected:
	tSchedulerTime NextTime = 0;
	uint32_t Offset;  // Offset to avoid all calls with similar period trigger in the same cycle.
	uint32_t Period;

  public:
	tSyncScheduler(bool Enable=false, uint32_t _Period=0, uint32_t _Offset=0);
	//void init(bool Enable=false, uint32_t _Period=0, uint32_t _Offset=0);

	inline void Disable() { if(IsEnabled()) NextTime=SchedulerDisabled; }
	inline void Enable() { if(IsDisabled()) UpdateNextTime(); }
	inline bool IsDisabled() const { return NextTime==SchedulerDisabled; }
	inline bool IsEnabled() const { return NextTime!=SchedulerDisabled; }

	inline bool IsTime() {
		if(HAL_GetTick()>NextTime) {
			UpdateNextTime();
			return true;
		}
		else {
			return false;
		}
	}
	inline tSchedulerTime Remaining() { return IsTime()?0:NextTime-HAL_GetTick(); }


	void SetPeriod(uint32_t _Period);
	void SetOffset(uint32_t _Offset);
	void SetPeriodAndOffset(uint32_t _Period, uint32_t _Offset);
	uint32_t GetOffset() const;
	uint32_t GetPeriod() const;
	tSchedulerTime GetNextTime() const;
	tSchedulerTime GetLastTime() const;

	void UpdateNextTime();

};




#endif /* SCHEDULER_HPP_ */


