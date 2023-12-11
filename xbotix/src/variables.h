
int mode = 1;

byte shoot_basket_col = 2;

byte shoot_position = 2;

#define leftStep 173
#define rightStep 175
#define BackStep 320
#define advance_distance 105

#define switch_Distance 70

#define turnBrake 15

#define leftBaseSpd 255
#define rightBaseSpd 245

#define blackThreashold 880
#define whiteThreshold 6000

#define servo_release_angle 90
#define servo_lock_angle 40

#define calibration_time 250 // calibrate time

float kp = 0.14;
float kd = 0.02;

float kp_white = 0.18;
float kd_white = 0.021;

float kp_copy = 0.3;
float kd_copy = 0.06;
#define double_check_distance 4

int input_distance_left = 15;
int input_distance_right = 15;
#define max_aligne_angle 2
#define center_band_width 1
int kp_align = 15;
int kd_align = 0.1;
int kp_reach = 13;
int kd_reach = 0.1;