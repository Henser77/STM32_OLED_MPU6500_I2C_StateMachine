#ifndef __KEY_H
#define __KEY_H

#define KEY_EVENT_NONE	0
#define	KEY_EVENT_SHORT	1
#define	KEY_EVENT_LONG 	2

void Key_Init(void);
uint8_t Key_Scan(void);


#endif

