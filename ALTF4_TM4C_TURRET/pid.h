#ifndef __PID_H__
#define __PID_H__

typedef struct PID
{
    int K;             // Gain for everything
    int time_constant; // For integral
    int error;         // Expected output - Actual Output
    int reset_register;
    int differential_error; 
    int dt;       // execution time of the loop
} PID;

int UpdatePID( PID *pid, int position );
int getProportionalComponent( PID *pid, int position );
int getIntegralComponent( PID *pid, int position );
int getDerivativeComponent( PID *pid, int position );

#endif
