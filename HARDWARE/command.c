#include "command.h"

u8 Is_GPS_Command = 0;
u8 Command_Index[3] = {0};
__align(32) u8 Command_Context[4] = {0};


