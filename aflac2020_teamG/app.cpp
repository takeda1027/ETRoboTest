//
//  app.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/04/28.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"

#include "crew.hpp"

Clock*      clock;
Captain*    captain;
Observer*   observer;
Radioman*   radioman;
Navigator*  activeNavigator = NULL;
uint8_t     state = ST_takingOff;

// a cyclic handler to activate a task
void task_activator(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    assert(ercd == E_OK || E_QOVR);
    /*
    if (ercd != E_OK) {
        _debug(syslog(LOG_NOTICE, "%08lu, act_tsk() returned %d", clock->now(), ercd));
    }
    */
}

// Observer's periodic task
void observer_task(intptr_t unused) {
    if (observer != NULL) observer->operate();
}

// Navigator's periodic task
void navigator_task(intptr_t unused) {
    if (activeNavigator != NULL) activeNavigator->operate();
}

// Radioman's resident task
void radioman_task(intptr_t unused) {    
    _debug(syslog(LOG_NOTICE, "%08u, radioman task ready", clock->now()));

    while (true) { // infinite loop
        if (radioman != NULL) radioman->operate();
    }
}

void main_task(intptr_t unused) {
    clock    = new Clock;
    captain  = new Captain;
    radioman = new Radioman;
    calibrator = new Calibrator;

    captain->takeoff();
    
    // sleep until being waken up
    ER ercd = slp_tsk();
    assert(ercd == E_OK);

    captain->land();

    delete calibrator;
    delete radioman;
    delete captain;
    delete clock;
    ext_tsk();
}
