#include "pid.h"

int UpdatePID( PID *pid, int position ){
    // Proportional Evaluation
    int p_drive = getProportionalComponent( pid, position );
    int i_drive = getIntegralComponent( pid, position );
    int d_drive = getDerivativeComponent( pid, position );

    return p_drive + i_drive + d_drive;
}

int getProportionalComponent( PID *pid, int position )
{
    return ( pid->K * pid->error );
}

int getIntegralComponent( PID *pid, int position )
{
    pid->reset_register += ( pid->K / (float)pid->time_constant ) * pid->error;
    return ( pid->K * pid->error + pid->reset_register );
}

int getDerivativeComponent( PID *pid, int position )
{
    return 0;
}
