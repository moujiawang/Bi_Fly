#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "stm32f10x.h"
#include "delay.h"
#include "nrf_protocol.h"
#include "DTU.h"
#include "motor.h"
#include "24l01.h" 	
#include <stdlib.h>
#include <string.h>
#include "upload_state_machine.h"
#include "IncPID.h"
#include "task.h"
#include "Data_map.h"
#include "eeprom.h"

void System_init(void);


#endif /* __SYSTEM_H */
