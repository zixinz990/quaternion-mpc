#pragma once

// control time related
#define MPC_UPDATE_PERIOD 5.0      // ms
#define LOW_LEVEL_CTRL_PERIOD 0.25 // ms
#define FEEDBACK_PERIOD 1.0        // ms

// robot constant
#define NUM_LEG 4
#define LEG_DOF 3
#define GRFS_DIM 12
#define NUM_DOF 12

#define UPPER_LEG_LENGTH 0.213
#define LOWER_LEG_LENGTH 0.213

// swing trajectory
#define FOOT_SWING_CLEARANCE1 0.0f
#define FOOT_SWING_CLEARANCE2 0.22f

#define FOOT_DELTA_X_LIMIT 0.5
#define FOOT_DELTA_Y_LIMIT 0.3
