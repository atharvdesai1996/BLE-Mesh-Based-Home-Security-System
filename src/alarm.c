/*
 * alarm.c
 *
 *  Created on: Nov 25, 2020
 *      Author: Joe Lopez
 */


#include "alarm.h"


// Flag for use on the friend node, indicating that an alarm state is in progress
int alarm_state;
// Flag for use on friend node, indicating that alarms are deactivated
int alarm_deactivate;
// Counter of seconds for which alarm is turned off
int ad_counter;


int get_ad_counter(void){
	return ad_counter;
}
void set_ad_counter(int cnt){
	ad_counter = cnt;
}

int get_alarm_state(void){
	return alarm_state;
}
void set_alarm_state(int a){
	alarm_state = a;
}

int get_alarm_deactivate(void){
	return alarm_deactivate;
}
void set_alarm_deactivate(int a){
	alarm_deactivate = a;
}
