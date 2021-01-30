/*
 * alarm.h
 * getters/setters for alarm deactivation variables
 *  Created on: Nov 25, 2020
 *      Author: Joe Lopez
 */

#ifndef SRC_ALARM_H_
#define SRC_ALARM_H_

int get_ad_counter(void);
void set_ad_counter(int);

int get_alarm_state(void);
void set_alarm_state(int);

int get_alarm_deactivate(void);
void set_alarm_deactivate(int);


#endif /* SRC_ALARM_H_ */
