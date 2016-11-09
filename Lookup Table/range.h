#ifndef RANGE_H
#define RANGE_H

#define PI 3.141592654f
#define sensor_error 0.1f
#define discrete_space (5.f*sensor_error)
#define theta_space (PI/72)

// error
#define ERROR_1 0.3f
#define ERROR_2 0.05f
#define ERROR_3 0.5f

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
// L_max equals to x of destination
#define W_min_L -2.75f
#define W_max_L 3.f 

//turn
#define X_min_T 5.f
#define X_max_T 50.f
#define Y_min_T 5.f
#define Y_max_T 50.f 
#define L_min_T 0.f
#define L_max_T 45.f
#define W_min_T 0.75f
#define W_max_T 50.f

//U_turn
#define X_min_U -10.f
#define X_max_U 50.f
#define Y_min_U 1.5f
#define Y_max_U 6.5f
#define L_min_U 0.f
#define L_max_U 50.f
#define W_min_U 0.75f
#define W_max_U 2.75f

#endif