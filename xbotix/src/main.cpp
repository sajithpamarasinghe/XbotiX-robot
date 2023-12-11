#include <Arduino.h>
#include <QTRSensors.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_TCS34725.h"
#include <NewPing.h>
#include <pin_define.h>
#include <variables.h>
#include <wire.h>

bool allBlack()
{
  qtr.readCalibrated(sensorValues);
  if ((sensorValues[0] > 500 &&
       sensorValues[1] > 500 &&
       sensorValues[2] > 500 &&
       sensorValues[3] > 500 &&
       sensorValues[4] > 500 &&
       sensorValues[5] > 500 &&
       sensorValues[6] > 500 &&
       sensorValues[7] > 500) ||
      (sensorValues[1] > 500 &&
       sensorValues[2] > 500 &&
       sensorValues[3] > 500 &&
       sensorValues[4] > 500 &&
       sensorValues[5] > 500 &&
       sensorValues[6] > 500 &&
       sensorValues[7] > 500) ||
      (sensorValues[0] > 500 &&
       sensorValues[1] > 500 &&
       sensorValues[2] > 500 &&
       sensorValues[3] > 500 &&
       sensorValues[4] > 500 &&
       sensorValues[5] > 500 &&
       sensorValues[6] > 500))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool allWhite()
{
  qtr.readCalibrated(sensorValues);
  if ((sensorValues[0] < 500 &&
       sensorValues[1] < 500 &&
       sensorValues[2] < 500 &&
       sensorValues[3] < 500 &&
       sensorValues[4] < 500 &&
       sensorValues[5] < 500 &&
       sensorValues[6] < 500 &&
       sensorValues[7] < 500) ||
      (sensorValues[1] < 500 &&
       sensorValues[2] < 500 &&
       sensorValues[3] < 500 &&
       sensorValues[4] < 500 &&
       sensorValues[5] < 500 &&
       sensorValues[6] < 500 &&
       sensorValues[7] < 500) ||
      (sensorValues[0] < 500 &&
       sensorValues[1] < 500 &&
       sensorValues[2] < 500 &&
       sensorValues[3] < 500 &&
       sensorValues[4] < 500 &&
       sensorValues[5] < 500 &&
       sensorValues[6] < 500))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool inWhiteLine()
{
  qtr.readCalibrated(sensorValues);
  if (sensorValues[0] > 500 && sensorValues[1] > 500 &&
      (sensorValues[2] < 500 ||
       sensorValues[3] < 500 ||
       sensorValues[4] < 500 ||
       sensorValues[5] < 500))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool inBlackLine()
{
  qtr.readCalibrated(sensorValues);
  if (sensorValues[0] < 500 && sensorValues[1] < 500 &&
      (sensorValues[2] > 500 ||
       sensorValues[3] > 500 ||
       sensorValues[4] > 500 ||
       sensorValues[5] > 500))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

bool WhitejunctionDetected()
{
  qtr.readCalibrated(sensorValues);
  if ((sensorValues[0] < 500 || sensorValues[7] < 500) &&
      (sensorValues[3] < 500 || sensorValues[4] < 500))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool WhiteLeftjunctionDetected()
{
  qtr.readCalibrated(sensorValues);
  if ((sensorValues[0] < 500) &&
      (sensorValues[3] < 500 || sensorValues[4] < 500))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool BlackjunctionDetected()
{
  qtr.readCalibrated(sensorValues);
  if (((sensorValues[0] > 500 || sensorValues[7] > 500) &&
       (sensorValues[3] > 500 || sensorValues[4] > 500)))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool WhiteRightLineDetected()
{
  qtr.readCalibrated(sensorValues);
  if (((sensorValues[7] < 500) &&
       (sensorValues[3] < 500 || sensorValues[4] < 500)))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool BlackRightLineDetected()
{
  qtr.readCalibrated(sensorValues);
  if (((sensorValues[7] > 500) &&
       (sensorValues[3] > 500 || sensorValues[4] > 500)))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool WhiteLeftLineDetected()
{
  qtr.readCalibrated(sensorValues);
  if (((sensorValues[0] < 500) &&
       (sensorValues[3] < 500 || sensorValues[4] < 500)))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool BlackLeftLineDetected()
{
  qtr.readCalibrated(sensorValues);
  if (((sensorValues[0] > 500) &&
       (sensorValues[3] > 500 || sensorValues[4] > 500)))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool WhiteFrontLineDetected()
{
  qtr.readCalibrated(sensorValues);
  if (((sensorValues[0] > 500 || sensorValues[7] > 500) &&
       (sensorValues[2] < 500 || sensorValues[3] < 500 || sensorValues[4] < 500 || sensorValues[5] < 500)))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool BlackFrontLineDetected()
{
  qtr.readCalibrated(sensorValues);
  if (((sensorValues[0] < 500 || sensorValues[7] < 500) &&
       (sensorValues[2] > 500 || sensorValues[3] > 500 || sensorValues[4] > 500 || sensorValues[5] > 500)))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool right_sensor_on_black()
{
  qtr.readCalibrated(sensorValues);
  if (sensorValues[7] > 500)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
bool left_sensor_on_black()
{
  qtr.readCalibrated(sensorValues);
  if (sensorValues[0] > 500)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void mdrive(int mleft, int mright)
{
  if (mleft > 0)
  {
    if (mleft > 255)
    {
      mleft = 255;
    }
    analogWrite(left_front, mleft);
    digitalWrite(left_back, LOW);
  }
  else if (mleft < 0)
  {
    if (mleft < -255)
    {
      mleft = -255;
    }
    digitalWrite(left_front, LOW);
    analogWrite(left_back, -1 * mleft);
  }
  else
  {
    digitalWrite(left_front, LOW);
    digitalWrite(left_back, LOW);
  }
  if (mright > 0)
  {
    if (mright > 255)
    {
      mright = 255;
    }
    analogWrite(right_front, mright);
    digitalWrite(right_back, LOW);
  }
  else if (mright < 0)
  {
    if (mright < -255)
    {
      mright = -255;
    }
    digitalWrite(right_front, LOW);
    analogWrite(right_back, -1 * mright);
  }
  else
  {
    digitalWrite(right_front, LOW);
    digitalWrite(right_back, LOW);
  }
}
float getDist_left_front_sonar()
{
  duration_left_front_sonar = left_front_sonar.ping();
  distance_left_front_sonar = (duration_left_front_sonar / 2) * 0.0343;
  return distance_left_front_sonar;
}
float getDist_left_back_sonar()
{
  duration_left_back_sonar = left_back_sonar.ping();
  distance_left_back_sonar = (duration_left_back_sonar / 2) * 0.0343;
  return distance_left_back_sonar;
}
float getDist_left_up_sonar()
{
  duration_left_up_sonar = left_up_sonar.ping();
  distance_left_up_sonar = (duration_left_up_sonar / 2) * 0.0343;
  return distance_left_up_sonar;
}
float getDist_right_front_sonar()
{
  duration_right_front_sonar = right_front_sonar.ping();
  distance_right_front_sonar = (duration_right_front_sonar / 2) * 0.0343;
  return distance_right_front_sonar;
}
float getDist_right_back_sonar()
{
  duration_right_back_sonar = right_back_sonar.ping();
  distance_right_back_sonar = (duration_right_back_sonar / 2) * 0.0343;
  return distance_right_back_sonar;
}
float getDist_center_up_sonar()
{
  duration_center_up_sonar = center_up_sonar.ping();
  distance_center_up_sonar = (duration_center_up_sonar / 2) * 0.0343;
  return distance_center_up_sonar;
}
float getDist_center_down_sonar()
{
  duration_center_down_sonar = center_down_sonar.ping();
  distance_center_down_sonar = (duration_center_down_sonar / 2) * 0.0343;
  return distance_center_down_sonar;
}

int encorderCount = 0;
void countEncorderDist()
{
  encorderCount++;
}

float lastErr;
void followBlackLine()
{
  int black_position = qtr.readLineBlack(sensorValues);
  int error = black_position - 3500;
  float dif = error * kp + (error - lastErr) * kd;
  mdrive(leftBaseSpd + dif, rightBaseSpd - dif);

  /*display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(error);
  display.setCursor(0, 20);
  display.println(dif);
  display.setCursor(0, 40);
  display.println(leftBaseSpd + dif);
  display.setCursor(50, 40);
  display.println(rightBaseSpd - dif);
  display.display();
  lastErr = error;*/
}
void followBlackLine_copy()
{
  int black_position = qtr.readLineBlack(sensorValues);
  int error = black_position - 3500;
  float dif = error * kp_copy + (error - lastErr) * kd_copy;
  mdrive(leftBaseSpd + dif, rightBaseSpd - dif);

  /* display.clearDisplay();
   display.setTextSize(1);
   display.setCursor(0, 0);
   display.println(error);
   display.setCursor(0, 20);
   display.println(dif);
   display.setCursor(0, 40);
   display.println(leftBaseSpd + dif);
   display.setCursor(50, 40);
   display.println(rightBaseSpd - dif);
   display.display();*/
  lastErr = error;
}
void followWhiteLine()
{
  int white_position = qtr.readLineWhite(sensorValues);
  int error = white_position - 3500;
  float dif = error * kp_white + (error - lastErr) * kd_white;
  mdrive(leftBaseSpd + dif, rightBaseSpd - dif);

  /*  display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(white_position);
    display.setCursor(0, 20);
    display.println(dif);
    display.setCursor(0, 40);
    display.println(leftBaseSpd + dif);
    display.setCursor(50, 40);
    display.println(rightBaseSpd - dif);
    display.display();
    lastErr = error;*/
}

void advance(int dis)
{
  attachInterrupt(digitalPinToInterrupt(encorderPin), countEncorderDist, CHANGE);
  if (dis > 0)
  {
    while (encorderCount < dis)
    {
      mdrive(leftBaseSpd, rightBaseSpd);
      delay(1);
    }
  }
  else
  {
    while (encorderCount < (dis * -1))
    {
      mdrive(-leftBaseSpd, -rightBaseSpd);
      delay(1);
    }
  }
  detachInterrupt(digitalPinToInterrupt(encorderPin));
  encorderCount = 0;
  mdrive(0, 0);
}

void advance_follow_line(int dis)
{
  attachInterrupt(digitalPinToInterrupt(encorderPin), countEncorderDist, CHANGE);

  while (encorderCount < dis)
  {
    followBlackLine();
  }
  detachInterrupt(digitalPinToInterrupt(encorderPin));
  encorderCount = 0;
  mdrive(0, 0);
}
void turnLeft()
{
  attachInterrupt(digitalPinToInterrupt(encorderPin), countEncorderDist, CHANGE);
  while (encorderCount < leftStep)
  {
    mdrive(-(leftBaseSpd), rightBaseSpd);
    delay(5);
  }
  mdrive(leftBaseSpd, -(rightBaseSpd));
  delay(turnBrake);
  detachInterrupt(digitalPinToInterrupt(encorderPin));
  encorderCount = 0;
  mdrive(0, 0);
}
void turnRight()
{
  attachInterrupt(digitalPinToInterrupt(encorderPin), countEncorderDist, CHANGE);
  while (encorderCount < rightStep)
  {
    mdrive(leftBaseSpd, -(rightBaseSpd));
    delay(5);
  }
  mdrive(-(leftBaseSpd), +rightBaseSpd);
  delay(turnBrake);
  detachInterrupt(digitalPinToInterrupt(encorderPin));
  encorderCount = 0;
  mdrive(0, 0);
}
void turnBack()
{
  attachInterrupt(digitalPinToInterrupt(encorderPin), countEncorderDist, CHANGE);
  while (encorderCount < BackStep)
  {
    mdrive(-(leftBaseSpd), rightBaseSpd);
    delay(5);
  }
  mdrive(leftBaseSpd, -(rightBaseSpd));
  delay(turnBrake);
  detachInterrupt(digitalPinToInterrupt(encorderPin));
  encorderCount = 0;
  mdrive(0, 0);
}
void foreward()
{
  mdrive(0, 0);
  delay(500);
}
void go_to_line_end()
{
  while (!allWhite())
  {
    followBlackLine();
  }
  mdrive(0, 0);
}
void go_to_white_line_end()
{
  while (!allBlack())
  {
    followWhiteLine();
  }
  mdrive(0, 0);
}
void go_to_next_black_junction()
{
  while (!BlackjunctionDetected())
  {
    followBlackLine();
  }
  mdrive(0, 0);
}
void go_to_next_white_junction()
{
  while (!WhitejunctionDetected())
  {
    followWhiteLine();
  }
  mdrive(0, 0);
}
void go_to_next_white_left_junction()
{
  while (!WhiteLeftjunctionDetected())
  {
    followWhiteLine();
  }
  mdrive(0, 0);
}

int A_position_colour_value = 0;
int B_position_colour_value = 0;
int C_position_colour_value = 0;
int x_position = 0;
int y_position = 0;
void calculate_position()
{
  x_position = (A_position_colour_value + B_position_colour_value + C_position_colour_value) % 6;
  if ((A_position_colour_value + C_position_colour_value) > 6)
  {
    y_position = (A_position_colour_value + C_position_colour_value) - 6;
  }
  else
  {
    y_position = (A_position_colour_value + C_position_colour_value);
  }
  display.clearDisplay();
  display.setTextSize(3);
  display.setCursor(20, 0);
  display.println(x_position);
  display.setCursor(50, 0);
  display.println(y_position);
  display.display();
}
bool limit_sw_pressed()
{
  bool limt = digitalRead(limitSwitch_pin);
  if (limt == 0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
void beep()
{
  analogWrite(buzzer, 250);
  delay(100);
  analogWrite(buzzer, 0);
  delay(100);
  analogWrite(buzzer, 250);
  delay(100);
  analogWrite(buzzer, 0);
  delay(100);
}
byte getColour()
{
  digitalWrite(colour_sensor_led_pin, HIGH); // colour sensor led
  delay(50);
  uint16_t r, g, b, c;
  delay(154); // Delay for one old integ. time period (to finish old reading)
  delay(615); // Delay for one new integ. time period (to allow new reading)
  tcs.getRawData(&r, &g, &b, &c);
  delay(10);
  digitalWrite(colour_sensor_led_pin, LOW);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(r);
  display.setCursor(0, 15);
  display.println(g);
  display.setCursor(0, 30);
  display.println(b);
  display.setCursor(0, 45);
  display.println(c);
  display.display();
  if (c > whiteThreshold)
  {
    return 1; // white
  }
  else if (c < blackThreashold)
  {
    return 5; // black
  }
  else
  {
    if (r > g && r > b)
    {
      return 2; // red
    }
    else if (g > r && g > b)
    {
      return 3; // green
    }
    else if (b > g && b > r)
    {
      return 4; // blue
    }
    return 0;
  }
}
byte get_basket_Colour()
{
  digitalWrite(colour_sensor_led_pin, HIGH); // colour sensor led
  delay(50);
  uint16_t r, g, b, c;
  delay(154); // Delay for one old integ. time period (to finish old reading)
  delay(615); // Delay for one new integ. time period (to allow new reading)
  tcs.getRawData(&r, &g, &b, &c);
  delay(10);
  digitalWrite(colour_sensor_led_pin, LOW);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(r);
  display.setCursor(0, 15);
  display.println(g);
  display.setCursor(0, 30);
  display.println(b);
  display.setCursor(0, 45);
  display.println(c);
  display.display();

  if (r > g && r > b)
  {
    return 2; // red
  }
  else if (g > r && g > b)
  {
    return 3; // green
  }
  else if (b > g && b > r)
  {
    return 4; // blue
  }
  return 0;
}
void showColouerRGBLed(int val)
{
  switch (val)
  {
  case 1: // white
    analogWrite(red_led, 255);
    analogWrite(green_led, 160);
    analogWrite(blue_led, 110);
    display.clearDisplay();
    display.setTextSize(3);
    display.setCursor(35, 0);
    display.println("WHITE");
    display.setCursor(55, 35);
    display.println(val);
    display.display();

    break;
  case 2: // red
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, LOW);
    digitalWrite(blue_led, LOW);
    display.clearDisplay();
    display.setTextSize(3);
    display.setCursor(35, 0);
    display.println("RED");
    display.setCursor(55, 35);
    display.println(val);
    display.display();
    break;
  case 3: // green
    digitalWrite(red_led, LOW);
    digitalWrite(green_led, HIGH);
    digitalWrite(blue_led, LOW);
    display.clearDisplay();
    display.setTextSize(3);
    display.setCursor(35, 0);
    display.println("GREEN");
    display.setCursor(55, 35);
    display.println(val);
    display.display();
    break;
  case 4: // blue
    digitalWrite(red_led, LOW);
    digitalWrite(green_led, LOW);
    digitalWrite(blue_led, HIGH);
    display.clearDisplay();
    display.setTextSize(3);
    display.setCursor(35, 0);
    display.println("BLUE");
    display.setCursor(55, 35);
    display.println(val);
    display.display();
    break;
  case 5: // black
    analogWrite(red_led, 1);
    analogWrite(green_led, 0);
    analogWrite(blue_led, 2);
    display.clearDisplay();
    display.setTextSize(3);
    display.setCursor(35, 0);
    display.println("BLACK");
    display.setCursor(55, 35);
    display.println(val);
    display.display();
    break;
  default:
    display.clearDisplay();
    digitalWrite(red_led, 0);
    digitalWrite(green_led, 0);
    digitalWrite(blue_led, 0);
    break;
  }
}
byte check_end_colour_plate()
{
  if ((getDist_center_up_sonar() > 0) && (getDist_center_up_sonar() < 15))
  {
    while (!limit_sw_pressed())
    {
      mdrive(leftBaseSpd - 100, rightBaseSpd - 110);
    }
    mdrive(-leftBaseSpd, -rightBaseSpd);
    delay(10);
    mdrive(0, 0);
    byte col = getColour();
    showColouerRGBLed(col);
    advance(-60);
    turnBack();
    return col;
  }
  else
  {
    turnBack();
    return 0;
  }
}
byte check_basket_colour()
{
  while (!limit_sw_pressed())
  {
    mdrive(leftBaseSpd - 90, rightBaseSpd - 100);
  }
  mdrive(-leftBaseSpd, -rightBaseSpd);
  delay(10);
  mdrive(0, 0);
  byte col = get_basket_Colour();
  showColouerRGBLed(col);
  advance(-60);
  turnBack();
  return col;
}

void shooter_release()
{

  int val = analogRead(pot);
  while (val > 500)
  {
    val = analogRead(pot);
    digitalWrite(dir2, 1);
    digitalWrite(dir1, 0);
  }
  digitalWrite(dir2, 0);
  digitalWrite(dir1, 0);
}
void shooter_lock()
{
  int val = analogRead(pot);
  while (val < 750)
  {
    val = analogRead(pot);
    digitalWrite(dir2, 0);
    digitalWrite(dir1, 1);
  }
  digitalWrite(dir2, 0);
  digitalWrite(dir1, 0);
}

float previous_error_Angle;
float previous_error_distance;

void align_wall_left()
{
  distance_left_front_sonar = getDist_left_front_sonar();
  distance_left_back_sonar = getDist_left_back_sonar();
  float error_angle = distance_left_back_sonar - distance_left_front_sonar;
  float derivativeA = error_angle - previous_error_Angle;
  int outputA = kp_align * error_angle + kd_align * derivativeA;
  previous_error_Angle = error_angle;
  int speedL = leftBaseSpd - 50 + outputA;
  int speedR = leftBaseSpd - 50 - outputA;
  mdrive(speedL, speedR);
  //  display.clearDisplay();
  //  display.setTextSize(1);
  //  display.setCursor(0, 0);
  //  display.println(distance1);
  //  display.setCursor(60, 0);
  //  display.println(distance2);
  //  display.setTextSize(2);
  //  display.setCursor(0, 50);
  //  display.println(angle);
  //
  //  display.display();
}
void reach_distance_left()
{
  distance_left_front_sonar = getDist_left_front_sonar();
  distance_left_back_sonar = getDist_left_back_sonar();
  float currentDistance = (distance_left_front_sonar + distance_left_back_sonar) / 2;
  float error_distance = currentDistance - input_distance_left;
  float derivative = error_distance - previous_error_distance;
  int outputD = kp_reach * error_distance + kd_reach * derivative;
  previous_error_distance = error_distance;

  int speedL = leftBaseSpd - 50 - outputD;
  int speedR = leftBaseSpd - 50 + outputD;
  mdrive(speedL, speedR);

  //  display.clearDisplay();
  //  display.setTextSize(1);
  //  display.setCursor(0, 0);
  //  display.println(distance1);
  //  display.setCursor(60, 0);
  //  display.println(distance2);
  //  display.setTextSize(2);
  //  display.setCursor(0, 30);
  //  display.println(errorD);
  //
  //  display.display();
}
int check_region_left(float read1, float read2)
{
  distance_left_front_sonar = getDist_left_front_sonar();
  distance_left_back_sonar = getDist_left_back_sonar();
  float distance = (distance_left_front_sonar + distance_left_back_sonar) / 2;
  if ((abs(distance - input_distance_left)) > center_band_width)
  {
    if (distance > input_distance_left)
    {
      return -1; // for left movement
    }
    else
    {
      return 1; // for right movement
    }
  }
  else
  {
    return 0; // inside region
  }
}
void follow_wall_left()
{
  distance_left_front_sonar = getDist_left_front_sonar();
  distance_left_back_sonar = getDist_left_back_sonar();
  byte region = check_region_left(distance_left_front_sonar, distance_left_back_sonar);
  float allign_angle = abs(distance_left_back_sonar - distance_left_front_sonar);

  if (region == 0)
  {
    align_wall_left();
    /* display.clearDisplay();
     display.setTextSize(3);
     display.setCursor(0, 0);
     display.println("Center");
     display.display();*/
  }
  else
  {
    /*display.clearDisplay();
    display.setTextSize(3);
    display.setCursor(0, 0);
    display.println("Err");
    display.display();*/

    if (allign_angle > max_aligne_angle)
    {
      align_wall_left();
    }
    else
    {
      reach_distance_left();
    }
  }
}

void align_wall_right()
{
  distance_right_front_sonar = getDist_right_front_sonar();
  distance_right_back_sonar = getDist_right_back_sonar();
  float error_angle = distance_right_back_sonar - distance_right_front_sonar;
  float derivativeA = error_angle - previous_error_Angle;
  int outputA = kp_align * error_angle + kd_align * derivativeA;
  previous_error_Angle = error_angle;
  int speedL = rightBaseSpd - 50 - outputA;
  int speedR = rightBaseSpd - 50 + outputA;
  mdrive(speedL, speedR);
  //  display.clearDisplay();
  //  display.setTextSize(1);
  //  display.setCursor(0, 0);
  //  display.println(distance1);
  //  display.setCursor(60, 0);
  //  display.println(distance2);
  //  display.setTextSize(2);
  //  display.setCursor(0, 50);
  //  display.println(angle);
  //
  //  display.display();
}
void reach_distance_right()
{
  distance_right_front_sonar = getDist_right_front_sonar();
  distance_right_back_sonar = getDist_right_back_sonar();
  float currentDistance = (distance_right_front_sonar + distance_right_back_sonar) / 2;
  float error_distance = currentDistance - input_distance_right;
  float derivative = error_distance - previous_error_distance;
  int outputD = kp_reach * error_distance + kd_reach * derivative;
  previous_error_distance = error_distance;

  int speedL = rightBaseSpd - 50 + outputD;
  int speedR = rightBaseSpd - 50 - outputD;
  mdrive(speedL, speedR);

  //  display.clearDisplay();
  //  display.setTextSize(1);
  //  display.setCursor(0, 0);
  //  display.println(distance1);
  //  display.setCursor(60, 0);
  //  display.println(distance2);
  //  display.setTextSize(2);
  //  display.setCursor(0, 30);
  //  display.println(errorD);
  //
  //  display.display();
}
int check_region_right(float read1, float read2)
{
  distance_right_front_sonar = getDist_right_front_sonar();
  distance_right_back_sonar = getDist_right_back_sonar();
  float distance = (distance_right_front_sonar + distance_right_back_sonar) / 2;
  if ((abs(distance - input_distance_right)) > center_band_width)
  {
    if (distance > input_distance_right)
    {
      return -1; // for right movement
    }
    else
    {
      return 1; // for right movement
    }
  }
  else
  {
    return 0; // inside region
  }
}
void follow_wall_right()
{
  distance_right_front_sonar = getDist_right_front_sonar();
  distance_right_back_sonar = getDist_right_back_sonar();
  byte region = check_region_right(distance_right_front_sonar, distance_right_back_sonar);
  float allign_angle = abs(distance_right_back_sonar - distance_right_front_sonar);

  if (region == 0)
  {
    align_wall_right();
    /*display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Center");
    display.display();*/
  }
  else
  {
    /*display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Err");
    display.display();*/
    if (allign_angle > max_aligne_angle)
    {
      align_wall_right();
    }
    else
    {
      reach_distance_right();
    }
  }
}

char path[30] = {};
char path_reverse[20] = {};
int pathLength;
void shortPath()
{
  int shortDone = 0;
  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'R')
  {
    pathLength -= 3;
    path[pathLength] = 'B';
    shortDone = 1;
  }
  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'S' && shortDone == 0)
  {
    pathLength -= 3;
    path[pathLength] = 'R';
    shortDone = 1;
  }
  if (path[pathLength - 3] == 'R' && path[pathLength - 1] == 'L' && shortDone == 0)
  {
    pathLength -= 3;
    path[pathLength] = 'B';
    shortDone = 1;
  }
  if (path[pathLength - 3] == 'S' && path[pathLength - 1] == 'L' && shortDone == 0)
  {
    pathLength -= 3;
    path[pathLength] = 'R';
    shortDone = 1;
  }
  if (path[pathLength - 3] == 'S' && path[pathLength - 1] == 'S' && shortDone == 0)
  {
    pathLength -= 3;
    path[pathLength] = 'B';
    shortDone = 1;
  }
  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'L' && shortDone == 0)
  {
    pathLength -= 3;
    path[pathLength] = 'S';
    shortDone = 1;
  }
  path[pathLength + 1] = 'D';
  path[pathLength + 2] = 'D';
  pathLength++;
}

bool left_line = 0;
bool right_line = 0;
bool front_line = 0;
bool end_line = 0;
bool white_square = 0;
void solve_maze_1()
{
  followWhiteLine();
  if (WhitejunctionDetected() || allWhite() || allBlack())
  {
    advance(8);
    if (WhiteLeftLineDetected())
    {
      left_line = 1;
    }
    if (WhiteRightLineDetected())
    {
      right_line = 1;
    }
    advance(advance_distance - 7);
    if (inWhiteLine())
    {
      front_line = 1;
    }
    if (allWhite())
    {
      white_square = 1;
    }
    if (allBlack())
    {
      end_line = 1;
    }
    /*  mdrive(0, 0);
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(left_line);
    display.setCursor(0, 10);
    display.println(right_line);
    display.setCursor(0, 20);
    display.println(front_line);
    display.setCursor(0, 30);
    display.println(end_line);
    display.setCursor(0, 40);
    display.println(white_square);
    display.display();
      delay(2000);*/
    advance(1);
    if (white_square == 1)
    {
      /*if (((getDist_left_front_sonar() > 1) && (getDist_left_front_sonar() < 20)) || ((getDist_right_front_sonar() > 1) && (getDist_right_front_sonar() < 20)))
      {
        turnBack();
        path[pathLength] = 'B';
        pathLength++;
        if (path[pathLength - 2] == 'B')
        {
          shortPath();
        }
        left_line = 0;
        right_line = 0;
        front_line = 0;
        end_line = 0;
        white_square = 0;
        return;
      }*/
      // else
      //{
      beep();
      turnBack();
      left_line = 0;
      right_line = 0;
      front_line = 0;
      end_line = 0;
      white_square = 0;
      mode = 12;
      return;
      //}
    }
    else if (left_line == 1)
    {
      turnLeft();
      left_line = 0;
      right_line = 0;
      front_line = 0;
      end_line = 0;
      white_square = 0;
      return;
    }
    else if (right_line == 1)
    {
      if (front_line == 1)
      {

        left_line = 0;
        right_line = 0;
        front_line = 0;
        end_line = 0;
        white_square = 0;
        return;
      }
      else
      {
        turnRight();
        left_line = 0;
        right_line = 0;
        front_line = 0;
        end_line = 0;
        white_square = 0;
        return;
      }
    }
    else if (end_line == 1)
    {
      turnBack();
      left_line = 0;
      right_line = 0;
      front_line = 0;
      end_line = 0;
      white_square = 0;
      return;
    }
  }
}
void solve_maze()
{
  followWhiteLine();
  if (WhitejunctionDetected() || allWhite() || allBlack())
  {
    advance(8);
    if (WhiteLeftLineDetected())
    {
      left_line = 1;
    }
    if (WhiteRightLineDetected())
    {
      right_line = 1;
    }
    advance(advance_distance - 7);
    if (inWhiteLine())
    {
      front_line = 1;
    }
    if (allWhite())
    {
      white_square = 1;
    }
    if (allBlack())
    {
      end_line = 1;
    }
    /* mdrive(0, 0);
     display.clearDisplay();
     display.setTextSize(1);
     display.setCursor(0, 0);
     display.println(left_line);
     display.setCursor(0, 10);
     display.println(right_line);
     display.setCursor(0, 20);
     display.println(front_line);
     display.setCursor(0, 30);
     display.println(end_line);
     display.setCursor(0, 40);
     display.println(white_square);
     display.display();
     delay(500);*/
    advance(1);
    if (white_square == 1)
    {
      /*if (((getDist_left_front_sonar() > 1) && (getDist_left_front_sonar() < 20)) || ((getDist_right_front_sonar() > 1) && (getDist_right_front_sonar() < 20)))
      {
        turnBack();
        path[pathLength] = 'B';
        pathLength++;
        if (path[pathLength - 2] == 'B')
        {
          shortPath();
        }
        left_line = 0;
        right_line = 0;
        front_line = 0;
        end_line = 0;
        white_square = 0;
        return;
      }*/
      // else
      //{
      beep();
      display.clearDisplay();
      display.println(pathLength);
      for (int i = 0; i < pathLength; i++)
      {
        display.setCursor((i * 10), 20);
        display.println(path[i]);
      }
      for (int i = 0; i < pathLength; i++)
      {
        path_reverse[i] = path[pathLength - i - 1];
      }
      for (int i = 0; i < pathLength; i++)
      {
        display.setCursor((i * 10), 40);
        display.println(path_reverse[i]);
      }
      display.display();
      left_line = 0;
      right_line = 0;
      front_line = 0;
      end_line = 0;
      white_square = 0;
      mode = 13;
      return;
      //}
    }
    else if (left_line == 1)
    {
      turnLeft();
      path[pathLength] = 'L';
      pathLength++;
      if (path[pathLength - 2] == 'B')
      {
        shortPath();
      }
      left_line = 0;
      right_line = 0;
      front_line = 0;
      end_line = 0;
      white_square = 0;
      return;
    }
    else if (right_line == 1)
    {
      if (front_line == 1)
      {
        path[pathLength] = 'S';
        pathLength++;
        if (path[pathLength - 2] == 'B')
        {
          shortPath();
        }
        left_line = 0;
        right_line = 0;
        front_line = 0;
        end_line = 0;
        white_square = 0;
        return;
      }
      else
      {
        turnRight();
        path[pathLength] = 'R';
        pathLength++;
        if (path[pathLength - 2] == 'B')
        {
          shortPath();
        }
        left_line = 0;
        right_line = 0;
        front_line = 0;
        end_line = 0;
        white_square = 0;
        return;
      }
    }
    else if (end_line == 1)
    {
      turnBack();
      path[pathLength] = 'B';
      pathLength++;
      if (path[pathLength - 2] == 'B')
      {
        shortPath();
      }
      left_line = 0;
      right_line = 0;
      front_line = 0;
      end_line = 0;
      white_square = 0;
      return;
    }
  }
}

void first_round()
{

  if (mode == 1) // button press
  {
    if (digitalRead(center_button) == 0)
    {
      delay(200);
      advance(150);
      mode = 2;
    }
  }
  if (mode == 2) // 1 stage colour recognition
  {
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    go_to_next_black_junction();
    advance(advance_distance);
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    while (!allWhite())
    {
      followBlackLine();
    }
    C_position_colour_value = check_end_colour_plate();
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    while (!allWhite())
    {
      followBlackLine();
    }
    B_position_colour_value = check_end_colour_plate();
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    while (!allWhite())
    {
      followBlackLine();
    }
    A_position_colour_value = check_end_colour_plate();
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    showColouerRGBLed(0);
    calculate_position();
    mode = 3;
  }
  if (mode == 3) // record path foreward
  {
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    go_to_next_black_junction();
    advance(40);
    mdrive(0, 0);
    bool end = 0;
    while (end == 0)
    {
      showColouerRGBLed(4);
      while (!allBlack())
      {
        followBlackLine_copy();
      }
      advance(advance_distance);
      if (inBlackLine())
      {
        end = 1;
      }
      else
      {
        advance(-advance_distance);
        end = 0;
      }
    }

    // advance(advance_distance);
    showColouerRGBLed(0);
    while (!BlackjunctionDetected())
    {
      followBlackLine_copy();
    }
    advance(advance_distance);
    while (!BlackjunctionDetected())
    {
      followBlackLine_copy();
    }
    advance(advance_distance - 20);
    turnBack();
    delay(500);
    advance(-(switch_Distance + 30));
    advance(switch_Distance);
    while (!BlackjunctionDetected())
    {
      followBlackLine_copy();
    }
    advance(advance_distance);
    while (!BlackjunctionDetected())
    {
      followBlackLine_copy();
    }
    advance(advance_distance);
    end = 0;
    while (end == 0)
    {
      showColouerRGBLed(2);
      while (!allBlack())
      {
        followBlackLine_copy();
      }
      advance(advance_distance);
      if (inBlackLine())
      {
        end = 1;
      }
      else
      {
        advance(-advance_distance);
        end = 0;
      }
    }
    // advance(advance_distance);
    showColouerRGBLed(0);
    mode = 4;
  }
  if (mode == 4) // record path backward
  {
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    go_to_next_black_junction();
    advance(advance_distance);
    go_to_next_black_junction();
    advance(advance_distance);
    go_to_next_black_junction();
    advance(advance_distance);
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    go_to_next_black_junction();
    advance(advance_distance);
    beep();
    mode = 5;
  }
  if (mode == 5) // go to cordinate x
  {
    for (int x = 0; x < x_position + 1; x++)
    {
      go_to_next_black_junction();
      advance(advance_distance);
    }
    turnRight();
    mode = 6;
  }
  if (mode == 6) // go to cordinate y
  {

    for (int y = 0; y < y_position; y++)
    {
      go_to_next_black_junction();
      advance(advance_distance);
    }

    mode = 7;
  }
  if (mode == 7) // finish
  {
    mdrive(0, 0);
    showColouerRGBLed(1);
    delay(200);
    showColouerRGBLed(0);
    delay(200);
  }
}

byte plate_colours[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte plate_col = 0;
byte basket_col = 0;
int colour_count[] = {0, 0, 0, 0, 0}; //  White,red,green,blue,black
bool boxes[3][3] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
};
int max_all = 0;
int box_count = 0;
float dis;
void second_round()
{
  if (mode == 1) // button press
  {
    if (digitalRead(center_button) == 0)
    {
      delay(200);
      advance(150);
      mode = 2;
    }
  }
  if (mode == 2) // 1 stage colour recognition
  {
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    go_to_next_black_junction();
    advance(advance_distance);
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    while (!allWhite())
    {
      followBlackLine();
    }
    C_position_colour_value = check_end_colour_plate();
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    while (!allWhite())
    {
      followBlackLine();
    }
    B_position_colour_value = check_end_colour_plate();
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    while (!allWhite())
    {
      followBlackLine();
    }
    A_position_colour_value = check_end_colour_plate();
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    showColouerRGBLed(0);
    plate_colours[18] = A_position_colour_value;
    plate_colours[19] = B_position_colour_value;
    plate_colours[20] = C_position_colour_value;
    mode = 3;
  }
  if (mode == 3) // record path foreward
  {
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    go_to_next_black_junction();
    advance(40);
    mdrive(0, 0);
    bool end = 0;
    while (end == 0)
    {
      showColouerRGBLed(4);
      while (!allBlack())
      {
        followBlackLine_copy();
      }
      advance(advance_distance);
      if (inBlackLine())
      {
        end = 1;
      }
      else
      {
        advance(-advance_distance);
        end = 0;
      }
    }

    // advance(advance_distance);
    showColouerRGBLed(0);
    while (!BlackjunctionDetected())
    {
      followBlackLine_copy();
    }
    advance(advance_distance);
    while (!BlackjunctionDetected())
    {
      followBlackLine_copy();
    }
    advance(advance_distance - 20);
    turnBack();
    delay(500);
    advance(-(switch_Distance + 30));
    advance(switch_Distance);
    while (!BlackjunctionDetected())
    {
      followBlackLine_copy();
    }
    advance(advance_distance);
    while (!BlackjunctionDetected())
    {
      followBlackLine_copy();
    }
    advance(advance_distance);
    end = 0;
    while (end == 0)
    {
      showColouerRGBLed(2);
      while (!allBlack())
      {
        followBlackLine_copy();
      }
      advance(advance_distance);
      if (inBlackLine())
      {
        end = 1;
      }
      else
      {
        advance(-advance_distance);
        end = 0;
      }
    }
    // advance(advance_distance);
    showColouerRGBLed(0);
    mode = 4;
  }
  if (mode == 4) // record path backward
  {
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();
    go_to_next_black_junction();
    advance(advance_distance);
    go_to_next_black_junction();
    advance(advance_distance);
    go_to_next_black_junction();
    advance(advance_distance);
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    go_to_next_black_junction();
    advance(advance_distance);
    mode = 5;
  }
  if (mode == 5) // grid
  {
    /*************************  0  *****************************/
    go_to_next_black_junction();
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      byte plate_col = check_end_colour_plate(); // chck 0 plate
      plate_colours[0] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   1   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10))
    {
      boxes[0][2] = 1;
      beep();
    }
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate(); // chck 0 plate
      plate_colours[1] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   2   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10))
    {
      boxes[0][1] = 1;
      beep();
    }
    else if ((getDist_right_front_sonar() < 70) && (getDist_right_front_sonar() > 45))
    {
      boxes[1][1] = 1;
      beep();
    }
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate();
      plate_colours[2] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   3   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10))
    {
      boxes[0][0] = 1;
      beep();
    }
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate();
      plate_colours[3] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   4  ***************************/
    go_to_next_black_junction();
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate();
      plate_colours[4] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   5  ***************************/
    go_to_line_end();
    plate_col = check_end_colour_plate();
    plate_colours[5] = plate_col;
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();

    /****************************   6   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10)) // 0,0 box
    {
      boxes[0][0] = 1;
      beep();
    }
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate(); // chck 0 plate
      plate_colours[6] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   7   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10))
    {
      boxes[1][0] = 1;
      beep();
    }
    else if ((getDist_right_front_sonar() < 70) && (getDist_right_front_sonar() > 45))
    {
      boxes[1][1] = 1;
      beep();
    }
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate();
      plate_colours[7] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   8   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10))
    {
      boxes[2][0] = 1;
      beep();
    }
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate();
      plate_colours[8] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   9  ***************************/
    go_to_next_black_junction();
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
      turnRight();
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate();
      plate_colours[9] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
    }
    /****************************   10   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10))
    {
      boxes[2][0] = 1;
      beep();
    }
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate(); // chck 0 plate
      plate_colours[10] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   11   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10))
    {
      boxes[2][1] = 1;
      beep();
    }
    else if ((getDist_right_front_sonar() < 70) && (getDist_right_front_sonar() > 45))
    {
      boxes[1][1] = 1;
      beep();
    }
    /*if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate();
      plate_colours[11] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }*/
    advance(advance_distance);
    /****************************   12   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10))
    {
      boxes[2][2] = 1;
      beep();
    }
    /*if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate();
      plate_colours[12] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }*/
    advance(advance_distance);
    /****************************   13  ***************************/
    go_to_next_black_junction();
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate();
      plate_colours[13] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   14  ***************************/
    go_to_line_end();
    plate_col = check_end_colour_plate();
    plate_colours[14] = plate_col;
    go_to_next_black_junction();
    advance(advance_distance);
    turnLeft();

    /****************************   15   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10)) // 0,0 box
    {
      boxes[2][2] = 1;
      beep();
    }
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate(); // chck 0 plate
      plate_colours[15] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   16   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10))
    {
      boxes[1][2] = 1;
      beep();
    }
    else if ((getDist_right_front_sonar() < 70) && (getDist_right_front_sonar() > 45))
    {
      boxes[1][1] = 1;
      beep();
    }
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate();
      plate_colours[16] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    /****************************   17   ***************************/
    go_to_next_black_junction();
    if ((getDist_right_front_sonar() < 45) && (getDist_right_front_sonar() > 10))
    {
      boxes[0][2] = 1;
      beep();
    }
    if (getDist_left_up_sonar() == 0) // check
    {
      advance(advance_distance);
    }
    else
    {
      advance(advance_distance);
      turnLeft();
      go_to_line_end();
      plate_col = check_end_colour_plate();
      plate_colours[17] = plate_col;
      go_to_next_black_junction();
      advance(advance_distance);
      turnLeft();
    }
    mode = 6;
  }
  if (mode == 6) // finish grid pass
  {
    go_to_next_black_junction();
    advance(advance_distance);
    turnRight();
    display.clearDisplay();
    for (int x = 0; x < 4; x++)
    {
      go_to_next_black_junction();
      advance(advance_distance);
    }
    turnRight();
    showColouerRGBLed(0);
    for (int x = 0; x < 4; x++)
    {
      go_to_next_black_junction();
      advance(advance_distance);
    }
    mode = 7;
  }
  if (mode == 7) // calculate
  {
    byte white_count = 0;
    byte red_count = 0;
    byte green_count = 0;
    byte blue_count = 0;
    byte black_count = 0;
    for (int i = 0; i < 21; i++)
    {
      if (plate_colours[i] == 1)
      {
        white_count++;
      }
      else if (plate_colours[i] == 2)
      {
        red_count++;
      }
      else if (plate_colours[i] == 3)
      {
        green_count++;
      }
      else if (plate_colours[i] == 4)
      {
        blue_count++;
      }
      else if (plate_colours[i] == 5)
      {
        black_count++;
      }
    }
    int m1 = max(white_count, red_count);
    int m2 = max(green_count, blue_count);
    int m3 = max(m1, m2);
    max_all = max(black_count, m3);

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        if (boxes[i][j] == 1)
        {
          box_count++;
        }
      }
    }
    display.clearDisplay();
    /*  display.setTextSize(1);
      display.setCursor(0, 0);
      display.println(white_count);
      display.setCursor(0, 10);
      display.println(red_count);
      display.setCursor(0, 20);
      display.println(green_count);
      display.setCursor(0, 30);
      display.println(blue_count);
      display.setCursor(0, 40);
      display.println(black_count);*/
    display.setTextSize(6);
    display.setCursor(0, 0);
    display.println(max_all);

    display.setCursor(50, 0);
    display.println(box_count);
    display.display();

    beep();

    mode = 8;
  }
  if (mode == 8) // go to cave
  {
    while (!allBlack())
    {
      followBlackLine();
    }
    mode = 9;
  }
  if (mode == 9) // wall follow
  {
    advance(300);
    // showColouerRGBLed(0);
    dis = getDist_center_down_sonar();
    while (!(dis > 1 && dis < 15))
    {
      follow_wall_left();
      dis = getDist_center_down_sonar();
    }
    turnRight();

    dis = getDist_center_down_sonar();
    while (!(dis > 1 && dis < 15))
    {
      follow_wall_left();
      dis = getDist_center_down_sonar();
    }
    turnRight();

    dis = getDist_center_down_sonar();
    while (!(dis > 1 && dis < 15))
    {
      mdrive(leftBaseSpd - 30, rightBaseSpd - 30);
      dis = getDist_center_down_sonar();
      // follow_wall_left();
    }
    mdrive(0, 0);
    // beep();
    // delay(500);
    ////////////////////////////////////
    // advance(645);
    // delay(500);
    /* dis = getDist_center_down_sonar();
     while (!(dis > 1 && dis < 15))
     {
       follow_wall_right();
       dis = getDist_center_down_sonar();
     }*/

    turnLeft();

    dis = getDist_center_down_sonar();
    while (!(dis > 1 && dis < 15))
    {
      follow_wall_right();
      dis = getDist_center_down_sonar();
    }
    mdrive(0, 0);
    turnLeft();

    dis = getDist_right_front_sonar();
    while ((dis > 1 && dis < 30))
    {
      follow_wall_right();
      dis = getDist_right_front_sonar();
    }
    mdrive(0, 0);
    delay(100);
    mode = 10;
  }
  if (mode == 10) // go to maze
  {
    int p = box_count + max_all;
    if (p >= 0 && p < 5)
    {
      shoot_basket_col = 2;
      showColouerRGBLed(2);
    }
    else if (p >= 5 && p < 10)
    {
      shoot_basket_col = 3;
      showColouerRGBLed(3);
    }
    else if (p >= 10)
    {
      shoot_basket_col = 4;
      showColouerRGBLed(4);
    }
    while (allWhite())
    {
      mdrive(leftBaseSpd, rightBaseSpd);
    }
    advance(50);
    mode = 11;
  }
  if (mode == 11) // go to maze start
  {
    solve_maze_1();
  }
  if (mode == 12) // solve maze
  {
    solve_maze();
  }
  if (mode == 13) // check basket colours and get shooting position
  {
    advance(100);
    go_to_next_white_junction();
    advance(advance_distance);
    go_to_white_line_end();
    basket_col = check_basket_colour();

    if (basket_col == shoot_basket_col)
    {
      go_to_next_white_junction();
      advance(advance_distance);
      shoot_position = 2;
      mode = 14;
      return;
    }

    go_to_next_white_junction();
    advance(advance_distance);
    turnLeft();
    go_to_next_white_junction();
    advance(advance_distance);
    turnLeft();

    go_to_white_line_end();
    basket_col = check_basket_colour();

    if (basket_col == shoot_basket_col)
    {
      go_to_next_white_junction();
      advance(advance_distance);
      turnRight();
      go_to_next_white_junction();
      advance(advance_distance);
      turnLeft();
      shoot_position = 1;
      mode = 14;
      return;
    }
    else
    {
      go_to_next_white_junction();
      advance(advance_distance);
      turnRight();
      go_to_next_white_junction();
      advance(advance_distance);
      turnLeft();
      shoot_position = 3;
      mode = 14;
      return;
    }
  }
  if (mode == 14) // go to optimize maze start
  {
    showColouerRGBLed(0);
    go_to_next_white_junction();
    advance(advance_distance + 100);
    beep();
    mode = 15;
  }
  if (mode == 15) // go optimized path
  {
    for (int i = 0; i < pathLength; i++)
    {
      go_to_next_white_junction();
      advance(advance_distance);
      if (path_reverse[i] == 'L')
      {
        turnRight();
      }
      else if (path_reverse[i] == 'R')
      {
        turnLeft();
      }
    }
    go_to_next_white_junction();
    advance(advance_distance + 100);
    mode = 16;
  }
  if (mode == 16) // go to sooting point and shoot
  {

    go_to_next_white_junction();
    advance(advance_distance);
    if (shoot_position == 1)
    {
      turnLeft();
      go_to_next_white_junction();
      advance(advance_distance);
      turnRight();
    }
    else if (shoot_position == 3)
    {
      turnRight();
      go_to_next_white_junction();
      advance(advance_distance);
      turnLeft();
    }
    go_to_next_white_junction();

    beep();
    delay(1000);
    shooter_release();
    for (int i = 0; i < 10; i++)
    {
      shooter_lock();
      delay(3000);
      shooter_release();
      delay(3000);
    }
    delay(10000000);
  }
}

void setup()
{
  shooter_release();
  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
  // shoot_servo.attach(servo_pin);
  pinMode(colour_sensor_led_pin, OUTPUT); // colour sensor led

  pinMode(left_front, OUTPUT);
  pinMode(left_back, OUTPUT);
  pinMode(right_front, OUTPUT);
  pinMode(right_back, OUTPUT);

  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);

  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(blue_led, OUTPUT);
  pinMode(buzzer, OUTPUT);

  pinMode(limitSwitch_pin, INPUT_PULLUP);
  pinMode(up_button, INPUT_PULLUP);
  pinMode(center_button, INPUT_PULLUP);
  pinMode(down_button, INPUT_PULLUP);

  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(SSD1306_WHITE);
  tcs.begin();

  display.clearDisplay(); // print ir calibrating
  display.setCursor(0, 5);
  display.println("ir calibrating");
  display.display();

  for (uint16_t i = 0; i < calibration_time; i++) // ir calibrating
  {
    qtr.calibrate();
  }

  display.clearDisplay();
  beep();
  digitalWrite(colour_sensor_led_pin, LOW);
  shooter_release();

  bool sr = digitalRead(down_button);
  while (sr == 1)
  {
    sr = digitalRead(down_button);
  }
  shooter_lock();
}

bool first = 0;
bool second = 0;
void loop()
{
  second_round();
}