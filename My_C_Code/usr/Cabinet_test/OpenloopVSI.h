#ifndef OPEN_LOOP_VSI
#define OPEN_LOOP_VSI



typedef struct OpenLoop_Command{
    int enable;
    int Num_inv;
    //store key variables and state information
    double freq;
    double amp;
    double command_volatge[3];
    bool is_init = 0;

} OpenLoop_Command;

OpenLoop_Command VSI_Openloop_command;

void OpenLoop_VSI(OpenLoop_Command *command_volatge);
void init_OpenLoop_Command(OpenLoop_Command *VSI_Openloop_command);
#endif
