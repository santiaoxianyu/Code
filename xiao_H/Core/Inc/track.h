#ifndef __TRACK_H__
#define __TRACK_H__

#include "headfile.h" 

#define All_Sensor1 track_read[0]==1&&track_read[1]==1&&track_read[2]==1&&track_read[3]==1&&track_read[4]==1&&track_read[5]==1&&track_read[6]==1&&track_read[7]==1
#define One_Sensor1 track_read[0]==0||track_read[1]==0||track_read[2]==0||track_read[3]==0||track_read[4]==0||track_read[5]==0||track_read[6]==0||track_read[7]==0
void Track_follow(void);
 
#endif
