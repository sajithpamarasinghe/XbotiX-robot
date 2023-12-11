
#define red_led 12
#define green_led 11
#define blue_led 10
#define buzzer 8
#define limitSwitch_pin 35
#define up_button 25
#define center_button 23
#define down_button 22

#define colour_sensor_led_pin 52

#define dir1 7
#define dir2 A11
#define pot A12
#define encorderPin 2

#define left_front 4
#define left_back 3
#define right_front 6
#define right_back 5

#define left_front_sonar_trig 51
#define left_front_sonar_echo 50
#define left_back_sonar_trig 49
#define left_back_sonar_echo 48
#define left_up_sonar_trig 31
#define left_up_sonar_echo 33

#define right_front_sonar_trig 43
#define right_front_sonar_echo 42
#define right_back_sonar_trig 47
#define right_back_sonar_echo 46

#define center_up_sonar_trig 36
#define center_up_sonar_echo 37
#define center_down_sonar_trig 41
#define center_down_sonar_echo 40

NewPing left_front_sonar(left_front_sonar_trig, left_front_sonar_echo, 50);
NewPing left_back_sonar(left_back_sonar_trig, left_back_sonar_echo, 50);
NewPing left_up_sonar(left_up_sonar_trig, left_up_sonar_echo, 60);/////////////////////////
NewPing right_front_sonar(right_front_sonar_trig, right_front_sonar_echo, 80);
NewPing right_back_sonar(right_back_sonar_trig, right_back_sonar_echo, 50);
NewPing center_up_sonar(center_up_sonar_trig, center_up_sonar_echo, 50);
NewPing center_down_sonar(center_down_sonar_trig, center_down_sonar_echo, 50);

float duration_left_front_sonar, distance_left_front_sonar;
float duration_left_back_sonar, distance_left_back_sonar;
float duration_left_up_sonar, distance_left_up_sonar;
float duration_right_front_sonar, distance_right_front_sonar;
float duration_right_back_sonar, distance_right_back_sonar;
float duration_center_up_sonar, distance_center_up_sonar;
float duration_center_down_sonar, distance_center_down_sonar;


Adafruit_SSD1306 display(128, 64, &Wire);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];



