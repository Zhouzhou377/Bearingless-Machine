



typedef struct OpenLoop_Command{
    int enable = 0;
    int Num_inv = 0;
    //store key variables and state information
    double freq = 0.0;
    double amp = 0.0;
    double command_volatge[3];

} OpenLoop_Command;

void OpenLoop_VSI(double freq, double amp, double *command_volatge);