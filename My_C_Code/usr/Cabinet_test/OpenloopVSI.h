#ifndef OPENLOOP_VSI_H
#define OPENLOOP_VSI_H
#include <stdbool.h>


typedef struct OpenLoop_Command{
    int enable;
    int Num_inv;
    //store key variables and state information
    double freq;
    double amp;
    double command_volatge[3];
    bool is_init;

} OpenLoop_Command;

extern OpenLoop_Command VSI_Openloop_command;

void OpenLoop_VSI(OpenLoop_Command *command_volatge);
OpenLoop_Command *init_OpenLoop_Command(void);
#endif //OPENLOOP_VSI_H
