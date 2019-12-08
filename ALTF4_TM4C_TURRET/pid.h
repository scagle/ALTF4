#ifndef __PID_H__
#define __PID_H__

typedef struct PID
{
    int Kp;             // Proportional Constant
    int Ki;             // Integral Constant
    int Kd;             // Derivative Constant
    int time_constant;  // For integral
    int error;          // Expected output - Actual Output
    int reset_register; // Accumulated error of integral
    int last_error;     // Track previous error for Derivative
} PID;

int UpdatePID( PID *pid, int position );
int getProportionalComponent( PID *pid, int position );
int getIntegralComponent( PID *pid, int position );
int getDerivativeComponent( PID *pid, int position );

#endif
