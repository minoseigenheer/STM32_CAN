/*
 * scheduler.cpp
 *
 *  Created on: 7 mrt. 2023
 *      Author: minoseigenheer
 */

#include "scheduler.hpp"


tSyncScheduler::tSyncScheduler(bool Enable, uint32_t _Period, uint32_t _Offset) :
		Offset(_Offset),
		Period(_Period) {

	if ( Enable ) {
		UpdateNextTime();
	} else {
		Disable();
	}
}


void tSyncScheduler::SetPeriod(uint32_t _Period) {
	Period=_Period;
	UpdateNextTime();
}
void tSyncScheduler::SetOffset(uint32_t _Offset) {
	Offset=_Offset;
	UpdateNextTime();
}
void tSyncScheduler::SetPeriodAndOffset(uint32_t _Period, uint32_t _Offset) {
	Period=_Period;
	Offset=_Offset;
	UpdateNextTime();
}
uint32_t tSyncScheduler::GetOffset() const { return Offset; }
uint32_t tSyncScheduler::GetPeriod() const { return Period; }
tSchedulerTime tSyncScheduler::GetNextTime() const { return NextTime; }
tSchedulerTime tSyncScheduler::GetLastTime() const { return NextTime - Period; }

void tSyncScheduler::UpdateNextTime() {
	if ( Period==0 ) {
		Disable();
		return;
	}
	tSchedulerTime now=HAL_GetTick();
	if ( Offset>now ) { // start or roll-over
		NextTime=Offset;
	} else {
		tSchedulerTime n=(now-Offset)/Period;
		NextTime=Offset+(n+1)*Period;
		if (NextTime==SchedulerDisabled) { NextTime=0; } // Avoid NextTime becoming SchedulerDisabled just before overflow
	}
}
