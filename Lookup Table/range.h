#ifndef RANGE_H
#define RANGE_H

#define PI 3.141592654f
#define sensor_error 0.1f
#define discrete_space (5.f*sensor_error)
#define theta_space (PI/72)

// error
#define ERROR_1 0.1f
#define ERROR_2 0.05f
#define ERROR_3 0.2f
#define ERROR_4 1.f

// vehicle
#define KMAX 0.26f
#define KMIN -0.26f
#define W 1.551f
#define L_F_BA 3.069f //length between front and behind achse

// x longitudinal y lateral x>0 front y>0 left

//straight
#define X_min_S discrete_space
#define X_max_S 100.f

// flag=1 left turn; flag=-1 right turn
//lane_change
#define X_min_L 0.5f
#define X_max_L 100.f
#define Y_min_L 1.5f
#define Y_max_L 6.5f
#define THETA_max_L (PI/4)
#define THETA_min_L (-PI/4)
#define L_min_L 2.f
#define L_max_L 50.f
#define W_min_L -2.75f
#define W_max_L 3.f 

//turn
#define X_min_LT 5.f
#define X_max_LT 50.f
#define Y_min_LT 5.f
#define Y_max_LT 50.f 
#define THETA_min_LT (0.25f*PI)
#define THETA_max_LT (0.75f*PI)
#define L_min_LT 0.f
#define L_max_LT 45.f
#define W_min_LT 0.75f
#define W_max_LT 50.f
#define R_min_LT 0.75f
#define R_max_LT 10.f
#define X_min_RT -0.5f
#define X_max_RT -25.f
#define Y_min_RT -0.5f
#define Y_max_RT -25.f
#define THETA_min_RT (-0.25f*PI)
#define THETA_max_RT (-0.75f*PI)

#endif