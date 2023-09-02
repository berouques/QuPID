/*
The MIT License (MIT)

Copyright (c) 2023 La Société des Berouques <berouque@outlook.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#define VER_MAJOR 0          // release number (if you change this number, all saved settings will be discarded)
#define VER_MINOR 30         // new functions or changes to struct_config (if you change this number, all saved settings will be discarded)
#define VER_PATCH 98         // minor changes
#define HISTORY_SIZE 255     // 1-255
#define MAX6675_INTERVAL 500 // millisec; time between updates
#define MAX6675_CLK_PIN D0   // SCK
#define MAX6675_CS_PIN D1    // CS
#define MAX6675_DATA_PIN D2  // SO
#define HEATER_PWM_MAX 255
#define HEATER_PWM_FREQ 10 // Hz
#define HEATER_INVERTED_OUTPUT true
#define HEATER_SSR_PIN D4
#define ENC_BUTTON_PIN 3 // RX pin
#define ENC_CLK_PIN D6
#define ENC_DT_PIN D5
#define MAX7219_DIN D8 // NB: has no effect; hardcoded in the library TODO: add to constructor in the library
#define MAX7219_CLK D7 // NB: has no effect; hardcoded in the library TODO: add to constructor in the library
#define MAX7219_CS D3  // NB: has no effect; hardcoded in the library TODO: add to constructor in the library

// libraries
#include "Version.h"
#include <Arduino.h>
#include <Wire.h>
#include <QuickPID.h>
#include <LittleFS.h>
#include <GyverPortal.h>
#include <GyverMAX6675.h>
#include <PinButton.h>
#include <max7219.h>
#include <PID_AutoTune_v0.h>

struct struct_config
{
  char app_id[6];
  uint8_t ver_major;
  uint8_t ver_minor;
  uint16_t ver_patch;

  // startup mode
  uint8_t Default_mode;

  // write to serial port current setpoint/temperature/pid output
  bool Log_enabled;

  // default display brightness
  uint8_t Display_brightness;

  // show that MCU is not stuck
  bool Display_heartbeat;

  // wifi: hide network name
  bool AP_hidden;
  // wifi channel
  uint8_t AP_radio_channel;
  // wifi access point IP
  uint8_t AP_ip_addr[4];
  // wifi network mask
  uint8_t AP_ip_mask[4];
  // wifi DHCP gateway
  uint8_t AP_ip_gw[4];

  // multiplier
  float Sensor_multiplier;
  // deg C
  float Setpoint_min;
  // deg C
  float Setpoint_max;
  // deg C
  float Setpoint_default;

  // safety switch: turns off heater if temperature higher than this value
  uint16_t Heater_temp_safety_limit;

  // time between calls of PID controller
  uint16_t PID_interval;
  // gap between the setpoint and the actual temperature
  uint8_t PID_adaptive_threshold;
  // basic mode gain
  float Kp_default;
  // basic mode gain
  float Ki_default;
  // basic mode gain
  float Kd_default;
  // adaptive mode gain
  float Kp_conservative;
  // adaptive mode gain
  float Ki_conservative;
  // adaptive mode gain
  float Kd_conservative;
  // adaptive mode gain
  float Kp_aggressive;
  // adaptive mode gain
  float Ki_aggressive;
  // adaptive mode gain
  float Kd_aggressive;

  // used to define the maximum number of samples used to perform the test. To get an accurate representation of the curve, the suggested range is 200-500
  uint16_t Stuner_samples;
  // testTimeSec = pid_interval * samples
  uint32_t Stuner_testTimeSec;
  // used to provide additional settling time prior to starting the test
  uint32_t Stuner_settleTimeSec;
  // initial control output value which is used for the settleTimeSec duration and sample 0
  float Stuner_outputStart;
  // stepped output value used for sample 1 to test completion
  float Stuner_outputStep;
  uint8_t Stuner_debounce;

  // program: array of delays and setpoints
  int32_t Program1[8];

  // program: temperature range (in percent) for moving to the next program step
  uint8_t Program1_temp_threshold;

  // program: active tasks
  bool Program1_active[8]; // TODO: bitmask and uint8

  // program: turn off heater after finishing
  bool Program1_turn_off;
};

// globals
#define MODE_BASIC 0
#define MODE_ADAPTIVE 1
#define MODE_PROGRAM1 2
#define MODE_PASSWORD 3
#define MODE_AUTOTUNER 4
#define MODE_ERROR 255
const char *Mode_name[] = {"BASIC", "ADAPTIVE", "PROGRAM", "Display WiFi password", "AUTOTUNER"};
const char *History_plot_names[] = {"Setpoint", "Temperature", "PWM"};
const char *Atuner_plot_names[] = {"SV", "PV", "PWM"};

// active configuration
struct_config Conf;

// current operating mode of the device
uint8_t Mode_current;

// previous operating mode of the device
uint8_t Mode_prev;

// time of the next call of the PID controller
unsigned long Duty_loop_time = 0;

// setpoint value for the web UI
int Plot_setpoint = 0;

// temperature value for the web UI
int Plot_sensor = 0;

// PWM value for the web UI
int Plot_pid = 0;

// circular buffer for operating values (setpoint, temperature, PWM)
int16_t History_arr[3][HISTORY_SIZE];

// time of the next refresh of the History web page
unsigned long History_live_time = 0;

// pause between refreshes of the History page
unsigned long History_live_interval = 2500;

// Wifi network password (generated based on MAC address in setup())
char AP_password[9];

// actual screen brightness
uint8_t Display_brightness = 10;

// next screen refresh time
unsigned long Display_update_time = 0;
int Display_heartbeat_increment = -1;

// previous encoder CLK value (set in the interrupt handler)
bool Encoder_CLK_prev = false;

// previous encoder DT value (set in the interrupt handler)
bool Encoder_DT_prev = false;

// current encoder value. used directly as a setpoint (set in the interrupt handler).
uint16_t Encoder_value = 0;

// latest temperature sensor value, after multiplication by Conf.Setpoint_multiplier
float Sensor_corrected = 0;

// latest temperature sensor value
float Sensor_raw = 0;

// time of the next request to the temperature sensor
unsigned long Sensor_update_time = 0;

// if false, the heater will not be turned on
bool Heater_enabled = true;

// current temperature for PID controller
float Pid_input;

// heater power output from the PID controller
float Pid_output;

// setpoint input to the PID controller
float Pid_setpoint;

double Atune_input = 0;
double Atune_output = 0;
double Atune_setpoint = 80;
uint8_t ATuneModeRemember = 2;
bool Atune_tuning = false;
double aTuneStep = HEATER_PWM_MAX / 2, aTuneNoise = 1, aTuneStartValue = HEATER_PWM_MAX / 2;
unsigned int aTuneLookBack = 20;

// current step for "program mode"
uint8_t Program_step = 0;

// current setpoint for "program mode"
float Program_setpoint;

// delay for "program mode"
double Program_remaining_delay = 0;
unsigned long Program_millis_prev = 0;
bool Program_is_waiting = false;

PinButton encoder_button(ENC_BUTTON_PIN);
MAX7219 max7219;
QuickPID pid_controller(&Pid_input, &Pid_output, &Pid_setpoint);
PID_ATune aTune(&Atune_input, &Atune_output);
GyverMAX6675<MAX6675_CLK_PIN, MAX6675_DATA_PIN, MAX6675_CS_PIN> max6675;
GyverPortal webui;

//  ######  ######## ##     ## ######## ########
// ##    ##    ##    ##     ## ##       ##
// ##          ##    ##     ## ##       ##
//  ######     ##    ##     ## ######   ######
//       ##    ##    ##     ## ##       ##
// ##    ##    ##    ##     ## ##       ##
//  ######     ##     #######  ##       ##

void serial_report(const char *mode, float sv, float pv, int pwm)
{

  if (Conf.Log_enabled)
  {
    Serial.printf("time:%i, mode:%s, sv:%f, pv:%f, pwm:%i\n", int(millis() / 1000), mode, sv, pv, pwm);
  }
}

void update_plot_values()
{

  Plot_sensor = Sensor_corrected;

  switch (Mode_current)
  {
  case MODE_AUTOTUNER:
    Plot_setpoint = Atune_setpoint;
    break;
  case MODE_PROGRAM1:
    Plot_setpoint = Program_setpoint;
    break;
  default:
    Plot_setpoint = Encoder_value;
    break;
  }

  // if the heater is turned off, display 0 on the graph, even if PID gives positive values.
  if (!Heater_enabled)
  {
    Plot_pid = 0;
  }
  else
  {
    Plot_pid = Pid_output;
  }
}

void set_mode(uint8_t new_mode)
{
  if (new_mode != Mode_current)
  {
    Mode_prev = Mode_current;
    Mode_current = new_mode;
    Program_step = 0;
  }
}

void next_mode()
{
  Mode_prev = Mode_current;
  Mode_current++;
  if (Mode_current > MODE_PASSWORD)
  {
    Mode_current = MODE_BASIC;
  }
}

//  ######   #######  ##    ## ######## ####  ######        ######## #### ##       ########
// ##    ## ##     ## ###   ## ##        ##  ##    ##       ##        ##  ##       ##
// ##       ##     ## ####  ## ##        ##  ##             ##        ##  ##       ##
// ##       ##     ## ## ## ## ######    ##  ##   ####      ######    ##  ##       ######
// ##       ##     ## ##  #### ##        ##  ##    ##       ##        ##  ##       ##
// ##    ## ##     ## ##   ### ##        ##  ##    ##       ##        ##  ##       ##
//  ######   #######  ##    ## ##       ####  ######        ##       #### ######## ########

bool load_defaults()
{
  strncpy(Conf.app_id, "QuPID", 6);
  Conf.ver_major = VER_MAJOR;
  Conf.ver_minor = VER_MINOR;
  Conf.ver_patch = VER_PATCH;
  Conf.Default_mode = MODE_BASIC; // 0 default, 1 adaptive, 2 stuner, 3 autotuner
  Conf.Log_enabled = true;
  Conf.Display_brightness = 10;
  Conf.Display_heartbeat = true;
  Conf.Sensor_multiplier = 1.0; // multiplier
  Conf.Setpoint_min = 0.0;      // deg C
  Conf.Setpoint_max = 300.0;    // deg C
  Conf.Setpoint_default = 0.0;  // deg C
  Conf.Heater_temp_safety_limit = 350;
  Conf.PID_interval = 1000;
  Conf.PID_adaptive_threshold = 50;
  Conf.Kp_default = 4.0;
  Conf.Ki_default = 1.0;
  Conf.Kd_default = 2.0;
  Conf.Kp_conservative = 1.0;
  Conf.Ki_conservative = 0.5;
  Conf.Kd_conservative = 4.0;
  Conf.Kp_aggressive = 15.0;
  Conf.Ki_aggressive = 1.0;
  Conf.Kd_aggressive = 0.0;

  Conf.Stuner_samples = 60;                       // used to define the maximum number of samples used to perform the test. To get an accurate representation of the curve, the suggested range is 200-500
  Conf.Stuner_testTimeSec = 60;                   // runPid interval = testTimeSec / samples
  Conf.Stuner_settleTimeSec = 1;                  // used to provide additional settling time prior to starting the test
  Conf.Stuner_outputStart = 0;                    // initial control output value which is used for the settleTimeSec duration and sample 0
  Conf.Stuner_outputStep = (HEATER_PWM_MAX / 20); // stepped output value used for sample 1 to test completion
  Conf.Stuner_debounce = 1;

  Conf.AP_hidden = false;
  Conf.AP_radio_channel = 3;

  Conf.AP_ip_addr[0] = 192;
  Conf.AP_ip_addr[1] = 168;
  Conf.AP_ip_addr[2] = 190;
  Conf.AP_ip_addr[3] = 1;
  Conf.AP_ip_gw[0] = 0;
  Conf.AP_ip_gw[1] = 0;
  Conf.AP_ip_gw[2] = 0;
  Conf.AP_ip_gw[3] = 0;
  Conf.AP_ip_mask[0] = 255;
  Conf.AP_ip_mask[1] = 255;
  Conf.AP_ip_mask[2] = 255;
  Conf.AP_ip_mask[3] = 0;

  Conf.Program1_turn_off = true;
  Conf.Program1_temp_threshold = 2;

  Conf.Program1_active[0] = false;
  Conf.Program1_active[1] = false;
  Conf.Program1_active[2] = false;
  Conf.Program1_active[3] = false;
  Conf.Program1_active[4] = false;
  Conf.Program1_active[5] = false;
  Conf.Program1_active[6] = false;
  Conf.Program1_active[7] = false;

  Conf.Program1[0] = 0;
  Conf.Program1[1] = 0;
  Conf.Program1[2] = 0;
  Conf.Program1[3] = 0;
  Conf.Program1[4] = 0;
  Conf.Program1[5] = 0;
  Conf.Program1[6] = 0;
  Conf.Program1[7] = 0;

  return true;
}

bool save_config(String file_name = "config")
{

  int arrsize = sizeof(struct_config);
  char buffer[arrsize];
  memcpy(&buffer, &Conf, arrsize);

  if (LittleFS.exists(file_name))
  {
    LittleFS.remove(file_name);
  }

  File this_file = LittleFS.open(file_name, "w");
  if (!this_file)
  {
    // failed to open the file, return false
    return false;
  }
  int bytesWritten = this_file.write(buffer, arrsize);

  if (bytesWritten == 0)
  {
    // write failed
    return false;
  }

  this_file.close();
  return true;
}

// load configuration from the internal flash, OR load default values
bool load_config(String file_name = "config")
{
  int arrsize = sizeof(struct_config);
  char buffer[arrsize];
  struct_config config;

  File this_file = LittleFS.open(file_name, "r");

  if (!this_file)
  {
    Serial.println("load_config(): can't open file.");
    return false;
  }

  // read file and cast into struct
  this_file.readBytes(buffer, arrsize);
  this_file.close();
  memcpy(&config, &buffer, arrsize);

  // validation
  if ((config.ver_major != VER_MAJOR) || (config.ver_minor != VER_MINOR))
  {
    // loading error, invalid config header
    return false;
  }
  else
  {
    Conf = config;
    return true;
  }
}

// ########  ####  ######  ########  ##          ###    ##    ##
// ##     ##  ##  ##    ## ##     ## ##         ## ##    ##  ##
// ##     ##  ##  ##       ##     ## ##        ##   ##    ####
// ##     ##  ##   ######  ########  ##       ##     ##    ##
// ##     ##  ##        ## ##        ##       #########    ##
// ##     ##  ##  ##    ## ##        ##       ##     ##    ##
// ########  ####  ######  ##        ######## ##     ##    ##

void init_display()
{
  max7219.Begin();
  max7219.MAX7219_SetBrightness(Conf.Display_brightness);
  max7219.Clear();
}

void display_heartbeats()
{

  if (Conf.Display_heartbeat)
  {
    Display_brightness += Display_heartbeat_increment;
    if ((Display_brightness <= 0))
    {
      Display_heartbeat_increment = 1;
      //   Display_heartbeat_increment = -Display_heartbeat_increment;
    }
    else if (Display_brightness >= Conf.Display_brightness)
    {
      Display_heartbeat_increment = -1;
    }
  }
  else
  {
    Display_brightness = Conf.Display_brightness;
  }

  max7219.MAX7219_SetBrightness(Display_brightness);
}

void update_display(int setpoint, int actual_temp)
{
  // update display 10 times per second
  if (Display_update_time < millis())
  {
    Display_update_time = millis() + 100;

    display_heartbeats();

    char text[9];

    if (!Heater_enabled)
    {
      sprintf(text, "OFF %4i", actual_temp);
      max7219.DisplayText(text, 0); // 0 left justified, 1 right justified
      return;
    }

    switch (Mode_current)
    {
    case MODE_BASIC:
      sprintf(text, "%4i%4i", setpoint, actual_temp);
      break;
    case MODE_ADAPTIVE:
      sprintf(text, "A%3i%4i", setpoint, actual_temp);
      break;
    case MODE_PROGRAM1:
      // limit program step with 2 digits
      if (Program_step > 99)
      {
        sprintf(text, "P_ER%4i", actual_temp);
      }
      else
      {
        sprintf(text, "P_%2i%4i", Program_step, actual_temp);
      }

      break;

    case MODE_PASSWORD:
      // show WIFI password
      sprintf(text, "%s", AP_password);
      break;

    case MODE_AUTOTUNER:
      sprintf(text, "TUNE%4i", actual_temp);
      break;

    case MODE_ERROR:
    default:
      sprintf(text, "ERR %4i", actual_temp);
      break;
    }

    max7219.DisplayText(text, 0); // 0 left justified, 1 right justified

  } // timer
}

//  ######  ######## ##    ##  ######   #######  ########
// ##    ## ##       ###   ## ##    ## ##     ## ##     ##
// ##       ##       ####  ## ##       ##     ## ##     ##
//  ######  ######   ## ## ##  ######  ##     ## ########
//       ## ##       ##  ####       ## ##     ## ##   ##
// ##    ## ##       ##   ### ##    ## ##     ## ##    ##
//  ######  ######## ##    ##  ######   #######  ##     ##

void update_sensor()
{
  if (Sensor_update_time <= millis())
  {
    Sensor_update_time = millis() + MAX6675_INTERVAL;

    Sensor_raw = max6675.getTemp();
    Sensor_corrected = Sensor_raw * Conf.Sensor_multiplier;
  }
}

// ######## ##    ##  ######   #######  ########  ######## ########
// ##       ###   ## ##    ## ##     ## ##     ## ##       ##     ##
// ##       ####  ## ##       ##     ## ##     ## ##       ##     ##
// ######   ## ## ## ##       ##     ## ##     ## ######   ########
// ##       ##  #### ##       ##     ## ##     ## ##       ##   ##
// ##       ##   ### ##    ## ##     ## ##     ## ##       ##    ##
// ######## ##    ##  ######   #######  ########  ######## ##     ##

/**
 * @brief The interrupt on any change of the encoder CLK pin
 */
IRAM_ATTR void encoder_int()
{

  bool clk = digitalRead(ENC_CLK_PIN);
  bool dt = digitalRead(ENC_DT_PIN);

  if (clk && !Encoder_CLK_prev)
  {
    if (dt && !Encoder_DT_prev)
    {
      Encoder_value++;
    }
    else
    {
      Encoder_value--;
    }
  }

  Encoder_CLK_prev = clk;
  Encoder_DT_prev = dt;

  Encoder_value = constrain(Encoder_value, Conf.Setpoint_min, Conf.Setpoint_max);
}

void set_encoder_value(int new_setpoint)
{
  int pos = constrain(new_setpoint, Conf.Setpoint_min, Conf.Setpoint_max);
  Encoder_value = pos;
}

// ##     ## ########    ###    ######## ######## ########
// ##     ## ##         ## ##      ##    ##       ##     ##
// ##     ## ##        ##   ##     ##    ##       ##     ##
// ######### ######   ##     ##    ##    ######   ########
// ##     ## ##       #########    ##    ##       ##   ##
// ##     ## ##       ##     ##    ##    ##       ##    ##
// ##     ## ######## ##     ##    ##    ######## ##     ##

int set_heater(int pwm)
{
  // safety first
  if (Sensor_corrected >= Conf.Heater_temp_safety_limit)
  {
    // if the temperature is above the safety threshold, turn off the heater
    pwm = 0;
    // TODO: "OVERHEAT" mode
  }

  if (!Heater_enabled)
  {
    pwm = 0;
  }

  pwm = constrain(pwm, 0, HEATER_PWM_MAX);
  // optional: inverse input value
  pwm = HEATER_INVERTED_OUTPUT ? (HEATER_PWM_MAX - pwm) : pwm;
  analogWrite(HEATER_SSR_PIN, pwm);

  return pwm;
}

// ########  #### ########
// ##     ##  ##  ##     ##
// ##     ##  ##  ##     ##
// ########   ##  ##     ##
// ##         ##  ##     ##
// ##         ##  ##     ##
// ##        #### ########

void pid_default(float sv, float pv)
{
  // simple PID
  Pid_setpoint = sv;
  Pid_input = pv;
  pid_controller.SetTunings(Conf.Kp_default, Conf.Ki_default, Conf.Kd_default);
  pid_controller.SetMode(QuickPID::Control::automatic);
  pid_controller.Compute();
  set_heater(Pid_output);
  serial_report("BASIC", sv, pv, Pid_output);
}

void pid_adaptive(float sv, float pv)
{
  // adaptive PID

  Pid_setpoint = sv;
  Pid_input = pv;

  // adaptive PID gains
  // distance away from setpoint
  float gap = abs(Encoder_value - Sensor_corrected);
  if (gap < Conf.PID_adaptive_threshold)
  {
    // we're close to setpoint, use conservative tuning parameters
    pid_controller.SetTunings(Conf.Kp_conservative, Conf.Ki_conservative, Conf.Kd_conservative);
  }
  else
  {
    // we're far from setpoint, use aggressive tuning parameters
    pid_controller.SetTunings(Conf.Kp_aggressive, Conf.Ki_aggressive, Conf.Kd_aggressive);
  }

  pid_controller.SetMode(QuickPID::Control::automatic);
  pid_controller.Compute();
  set_heater(Pid_output);
  serial_report("ADAPTIVE", sv, pv, Pid_output);
}

void autotuner_save_mode()
{
  ATuneModeRemember = pid_controller.GetMode();
}

void autotuner_restore_mode()
{
  pid_controller.SetMode(ATuneModeRemember);
}

void autotuner_cancel()
{
  aTune.Cancel();
  Atune_tuning = false;
  autotuner_restore_mode();
  set_mode(MODE_BASIC);
}

void autotuner_start()
{
  Atune_output = aTuneStartValue;
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
  autotuner_restore_mode();
  Atune_tuning = true;
  set_mode(MODE_AUTOTUNER);
}

void autotuner_tick(float sv, float pv)
{
  Atune_setpoint = sv;
  Atune_input = pv;

  if (aTune.Runtime() != 0)
  {
    // we're done, set the tuning parameters
    Conf.Kp_default = aTune.GetKp();
    Conf.Ki_default = aTune.GetKi();
    Conf.Kd_default = aTune.GetKd();
    save_config();
    autotuner_restore_mode();
    set_mode(MODE_BASIC);
  }

  Pid_output = constrain(Atune_output, 0, 255);
  set_heater(Pid_output);
  serial_report("AUTOTUNER", sv, pv, Atune_output);
}

// ########  ########   #######   ######   ########     ###    ##     ##
// ##     ## ##     ## ##     ## ##    ##  ##     ##   ## ##   ###   ###
// ##     ## ##     ## ##     ## ##        ##     ##  ##   ##  #### ####
// ########  ########  ##     ## ##   #### ########  ##     ## ## ### ##
// ##        ##   ##   ##     ## ##    ##  ##   ##   ######### ##     ##
// ##        ##    ##  ##     ## ##    ##  ##    ##  ##     ## ##     ##
// ##        ##     ##  #######   ######   ##     ## ##     ## ##     ##

void program_pid(float pv)
{

  Pid_setpoint = Program_setpoint;
  Pid_input = pv;
  pid_controller.SetTunings(Conf.Kp_default, Conf.Ki_default, Conf.Kd_default);
  pid_controller.SetMode(QuickPID::Control::automatic);
  pid_controller.Compute();
  set_heater(Pid_output);
  serial_report("PROGRAM", Program_setpoint, pv, Pid_output);
}

void program_setpoint(float pv)
{
  program_pid(pv);

  // if the set temperature +-threshold% is reached, go to the next step
  if (abs(Conf.Program1[Program_step] - Pid_input) <= ((Pid_input / 100) * Conf.Program1_temp_threshold))
  {
    Program_step++;
  }
}

void program_delay(float pv)
{
  program_pid(pv);

  if (!Program_is_waiting)
  {
    Program_remaining_delay = Conf.Program1[Program_step];
    Program_is_waiting = true;
    Program_millis_prev = millis();
  }
  else if (Program_remaining_delay > 0)
  {
    // subtract time from the counter 
    Program_remaining_delay -= (millis() - Program_millis_prev) / 1000;
    Program_millis_prev = millis();
  }
  else if (Program_remaining_delay <= 0)
  {
    // waiting period is over
    Program_is_waiting = false;
    Program_step++;
  }
}

void program_tick(int program_id, float pv)
{
  // PROGRAM FINISHED
  if (Program_step >= 8)
  {
    if (Conf.Program1_turn_off)
    {
      set_heater(0);
      Program_setpoint = 0;
    }
    else
    {
      set_encoder_value(Program_setpoint);
      set_mode(MODE_BASIC);
      Program_setpoint = 0;
    }
    return;
  }

  // SKIP INACTIVE STEPS
  while (!Conf.Program1_active[Program_step])
  {
    Program_step++;
  }

  // even steps - setpoint, odd steps - delay
  if (Program_step % 2)
  {
    // SETPOINT
    Program_setpoint = Conf.Program1[Program_step];
    program_setpoint(pv);
  }
  else
  {
    // DELAY
    program_delay(pv);
  }
}

// ##      ## ######## ########  ##     ## ####
// ##  ##  ## ##       ##     ## ##     ##  ##
// ##  ##  ## ##       ##     ## ##     ##  ##
// ##  ##  ## ######   ########  ##     ##  ##
// ##  ##  ## ##       ##     ## ##     ##  ##
// ##  ##  ## ##       ##     ## ##     ##  ##
//  ###  ###  ######## ########   #######  ####

// create page to display in browser
void webui_build(GyverPortal &p)
{
  char page_title[32];
  sprintf(page_title, "%s v.%i.%i.%i", Conf.app_id, VER_MAJOR, VER_MINOR, VER_PATCH);

  GP.BUILD_BEGIN(GP_DARK);
  GP.PAGE_TITLE(page_title);
  GP.ONLINE_CHECK();

  GP.UI_MENU(page_title);
  GP.UI_LINK("/", "Home");
  GP.UI_LINK("/history", "History");
  GP.UI_LINK("/settings", "Settings");
  GP.UI_LINK("/autotuner", "Autotuner");
  GP.UI_LINK("/admin", "Admin");
  GP.UI_BODY();

  if (Mode_current == MODE_AUTOTUNER)
  {
    // page AUTOTUNER PROGRESS =============================================================
    // if current mode is autotuner, block everything except progress plot
    GP.UPDATE("Autotuner_progress_reload");
    GP.RELOAD("Autotuner_progress_reload");
    GP.LABEL_BLOCK("Autotuning is in progress");
    M_BLOCK(GP.AJAX_PLOT_DARK("Plot_ajax", Atuner_plot_names, 3, 255, 1000, 400, true););
    GP.BUTTON_MINI("Cancel_autotuner", "Cancel");
  }

  else if (webui.uri() == "/settings")
  {
    // page SETTINGS =============================================================

    GP.LABEL_BLOCK("Settings");

    GP.NAV_TABS("Prefs,PID,Misc, Program");

    // Prefs tab
    GP.NAV_BLOCK_BEGIN();

    GP.LABEL("Startup");
    M_BLOCK(
        GP.BOX_BEGIN();
        GP.LABEL("Default setpoint, °C");
        GP.SPINNER("Setpoint_default", Conf.Setpoint_default, Conf.Setpoint_min, Conf.Setpoint_max);
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.LABEL("Default mode");
        GP.SELECT("Default_mode", "Basic,Adaptive,Program", Conf.Default_mode);
        GP.BOX_END(););

    GP.LABEL("Safety");
    M_BLOCK(

        GP.BOX_BEGIN();
        GP.LABEL("Sensor multiplier");
        GP.NUMBER_F("Sensor_multiplier", "number", Conf.Sensor_multiplier, 5);
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.LABEL("Setpoint_min, °C");
        GP.SPINNER("Setpoint_min", Conf.Setpoint_min, 0, 1000);
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.LABEL("Setpoint_max, °C");
        GP.SPINNER("Setpoint_max", Conf.Setpoint_max, 0, 1000);
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.LABEL("Safety limit, °C", "", GP_RED_B);
        GP.SPINNER("Heater_temp_safety_limit", Conf.Heater_temp_safety_limit, 0, 1000);
        GP.BOX_END(););

    GP.NAV_BLOCK_END();

    // PID tab
    GP.NAV_BLOCK_BEGIN();
    GP.LABEL("PID controller");
    M_BLOCK(
        GP.BOX_BEGIN();
        GP.LABEL("PID interval, seconds");
        GP.SPINNER("PID_interval", (Conf.PID_interval / 1000), 1, 60);
        GP.BOX_END(););

    GP.LABEL("Basic mode");
    M_BLOCK(
        GP.BOX_BEGIN();
        GP.LABEL("Kp");
        GP.NUMBER_F("Kp_default", "number", Conf.Kp_default, 5);
        GP.LABEL("Ki");
        GP.NUMBER_F("Ki_default", "number", Conf.Ki_default, 5);
        GP.LABEL("Kd");
        GP.NUMBER_F("Kd_default", "number", Conf.Kd_default, 5);
        GP.BOX_END(););

    GP.LABEL("Adaptive mode");
    M_BLOCK(
        GP.BOX_BEGIN();
        GP.LABEL("Gap, °C");
        GP.SPINNER("PID_adaptive_threshold", Conf.PID_adaptive_threshold, 1, 255);
        GP.BOX_END();
        GP.HR();
        GP.LABEL("Conservative");
        GP.BOX_BEGIN();
        GP.LABEL("Kp");
        GP.NUMBER_F("Kp_conservative", "number", Conf.Kp_conservative, 5);
        GP.LABEL("Ki");
        GP.NUMBER_F("Ki_conservative", "number", Conf.Ki_conservative, 5);
        GP.LABEL("Kd");
        GP.NUMBER_F("Kd_conservative", "number", Conf.Kd_conservative, 5);
        GP.BOX_END();

        GP.LABEL("Aggressive");
        GP.BOX_BEGIN();
        GP.LABEL("Kp");
        GP.NUMBER_F("Kp_aggressive", "number", Conf.Kp_aggressive, 5);
        GP.LABEL("Ki");
        GP.NUMBER_F("Ki_aggressive", "number", Conf.Ki_aggressive, 5);
        GP.LABEL("Kd");
        GP.NUMBER_F("Kd_aggressive", "number", Conf.Kd_aggressive, 5);
        GP.BOX_END();

    );

    GP.NAV_BLOCK_END();

    // misc tab
    GP.NAV_BLOCK_BEGIN();
    M_BLOCK(
        GP.BOX_BEGIN();
        GP.LABEL("Serial data output");
        GP.SWITCH("Log_enabled", Conf.Log_enabled);
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.LABEL("Display brightness");
        GP.SPINNER("Display_brightness", Conf.Display_brightness, 0, 15);
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.LABEL("Display heartbeat");
        GP.SWITCH("Display_heartbeat", Conf.Display_heartbeat);
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.LABEL("WiFi hidden");
        GP.SWITCH("AP_hidden", Conf.AP_hidden);
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.LABEL("WiFi channel");
        GP.SPINNER("AP_radio_channel", Conf.AP_radio_channel, 1, 14);
        GP.BOX_END();

    );

    GP.NAV_BLOCK_END();

    // program tab
    GP.NAV_BLOCK_BEGIN();
    GP.LABEL("Program");
    M_BLOCK(
        for (int i = 0; i < 8; i++) {
          GP.BOX_BEGIN();
          if (i % 2)
          {
            GP.LABEL("Setpoint, °C");
          }
          else
          {
            GP.LABEL("Delay, sec");
          }
          GP.NUMBER(String("Program1_value/") + i, "", Conf.Program1[i]);
          GP.SWITCH(String("Program1_active/") + i, Conf.Program1_active[i]);
          GP.BOX_END();
        }

        GP.BOX_BEGIN();
        GP.LABEL("Turn off heater");
        GP.SWITCH("Program1_turn_off", Conf.Program1_turn_off);
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.LABEL("Setpoint match threshold, %");
        GP.SPINNER("Program1_temp_threshold", Conf.Program1_temp_threshold, 1, 100);
        GP.BOX_END();

    );
    GP.NAV_BLOCK_END();

    GP.BUTTON("save_settings", "SAVE TO FLASH");
    GP.BREAK();
    GP.SPAN("device will be rebooted after saving");
  }
  else if (webui.uri() == "/admin")
  {
    // page ADMIN ========================================================

    GP.LABEL_BLOCK("Admin");

    M_BLOCK(

        GP.BUTTON("Admin_defaults_save", "Reboot to defaults");
        GP.HR();
        GP.BUTTON("Admin_defaults", "Apply defaults");
        GP.HR();
        GP.BUTTON("Admin_reboot", "Reboot");
        GP.HR();
        GP.OTA_FIRMWARE();
        GP.OTA_FILESYSTEM(););
  }
  else if (webui.uri() == "/autotuner")
  {
    // page AUTOTUNER ========================================================

    GP.LABEL_BLOCK("Autotuner");
    GP.UPDATE("Autotuner_reload");

    GP.RELOAD("Autotuner_reload");

    GP.SPAN("Be prepared that this process may take quite a long time.");
    GP.BREAK();
    GP.SPAN("After the process is completed, the new values of Kp, Ki, Kd will be AUTOMATICALLY written to flash for BASIC MODE.");
    GP.JQ_UPDATE_BEGIN();

    M_BLOCK(
        GP.BOX_BEGIN();
        GP.LABEL("Test setpoint, °C");
        GP.SPINNER("Atune_setpoint", Atune_setpoint, Conf.Setpoint_min, Conf.Setpoint_max);
        GP.BOX_END();

        GP.BUTTON_MINI("start_autotuner", "Start autotuner");

    ); // m_block
  }
  else if (webui.uri() == "/history")
  {
    // page HISTORY ========================================================

    GP.UPDATE("History_live_reload");
    GP.RELOAD("History_live_reload");
    GP.LABEL_BLOCK("History");
    GP.SPAN("latest 255 samples; page updates automatically", GP_CENTER);

    GP.BOX_BEGIN();
    GP.LABEL("Kp");
    GP.NUMBER_F("Kp", "number", pid_controller.GetKp(), 5, "", true);
    GP.LABEL("Ki");
    GP.NUMBER_F("Ki", "number", pid_controller.GetKi(), 5, "", true);
    GP.LABEL("Kd");
    GP.NUMBER_F("Kd", "number", pid_controller.GetKd(), 5, "", true);
    GP.BOX_END();

    GP.PLOT_DARK<3, HISTORY_SIZE>("Plot_static", History_plot_names, History_arr);
  }
  else
  {
    // page HOME ==============================================================

    GP.JQ_SUPPORT_FILE(); // jquery support, file will be downloaded from LittleFS in flash (/gp_data/jquery.js)

    GP.LABEL_BLOCK("Home");

    M_BLOCK(
        GP.JQ_UPDATE_BEGIN();

        GP.BOX_BEGIN();
        GP.LABEL("Heater on");
        GP.SWITCH("Heater_enabled", Heater_enabled);
        GP.BOX_END();

        GP.HR();

        GP.BOX_BEGIN();
        GP.LABEL("Mode");
        GP.LABEL(Mode_name[Mode_current], "Mode_name", GP_CYAN);
        GP.BUTTON_MINI("Mode_next", ">>");
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.LABEL("PV, °C");
        GP.NUMBER_F("Pid_input", "current temperature", Pid_input, 2, "", true);
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.LABEL("SV, °C");
        GP.SPINNER("Encoder_value", Encoder_value, Conf.Setpoint_min, Conf.Setpoint_max);
        GP.BOX_END();

        GP.BOX_BEGIN();
        GP.BUTTON("encoder_minus", "-5°C");
        GP.BUTTON("encoder_plus", "+5°C");
        GP.BOX_END();

        GP.JQ_UPDATE_END(););

    GP.LABEL("Live data");
    GP.AJAX_PLOT_DARK("Plot_ajax", History_plot_names, 3, 255, 1000, 400, true);
  } // if uri

  GP.UI_END();
  GP.BUILD_END();
}

// ##      ## ######## ########  ##     ## ####         ###     ######  ######## ####  #######  ##    ##
// ##  ##  ## ##       ##     ## ##     ##  ##         ## ##   ##    ##    ##     ##  ##     ## ###   ##
// ##  ##  ## ##       ##     ## ##     ##  ##        ##   ##  ##          ##     ##  ##     ## ####  ##
// ##  ##  ## ######   ########  ##     ##  ##       ##     ## ##          ##     ##  ##     ## ## ## ##
// ##  ##  ## ##       ##     ## ##     ##  ##       ######### ##          ##     ##  ##     ## ##  ####
// ##  ##  ## ##       ##     ## ##     ##  ##       ##     ## ##    ##    ##     ##  ##     ## ##   ###
//  ###  ###  ######## ########   #######  ####      ##     ##  ######     ##    ####  #######  ##    ##

// event handler for Web UI
void webui_action(GyverPortal &p)
{

  if (webui.update("Plot_ajax"))
  {
    int answ[3];
    answ[0] = Plot_setpoint;
    answ[1] = Plot_sensor;
    answ[2] = Plot_pid;
    webui.answer(answ, 3);
  }

  if (webui.update("History_live_reload") && (History_live_time < millis()))
  {
    History_live_time = millis() + History_live_interval;
    webui.answer(1);
  }

  if (p.click("Mode_next"))
  {
    next_mode();
  }

  if (p.click("Heater_enabled"))
  {
    Heater_enabled = p.getBool("Heater_enabled");
  }

  if (p.click("Encoder_value"))
  {
    set_encoder_value(p.getInt("Encoder_value"));
  }

  if (p.click("encoder_plus"))
  {
    set_encoder_value(Encoder_value + 5);
  }

  if (p.click("encoder_minus"))
  {
    set_encoder_value(Encoder_value - 5);
  }

  webui.clickInt("Atune_setpoint", Atune_setpoint);

  if (p.click("start_autotuner"))
  {
    // TODO webui indicator of autotuner
    // TODO display mode etc
    autotuner_start();
  }

  if (p.click("Cancel_autotuner"))
  {
    autotuner_cancel();
  }

  if (webui.update("Autotuner_progress_reload") && Mode_current != MODE_AUTOTUNER)
  {
    webui.answer(1);
  }

  if (webui.update("Autotuner_reload") && Mode_current == MODE_AUTOTUNER)
  {
    // force browser refresh to lock user on the page "Autotuner in progress"
    webui.answer(1);
  }

  // program
  webui.clickBool("Program1_turn_off", Conf.Program1_turn_off);
  webui.clickInt("Program1_temp_threshold", Conf.Program1_temp_threshold);

  if (webui.clickSub("Program1_value"))
  {
    int idx = webui.clickNameSub(1).toInt();
    Conf.Program1[idx] = webui.getInt();
  }
  if (webui.clickSub("Program1_active"))
  {
    int idx = webui.clickNameSub(1).toInt();
    Conf.Program1_active[idx] = webui.getBool();
  }

  // prefs
  webui.clickInt("Setpoint_default", Conf.Setpoint_default);
  webui.clickInt("Default_mode", Conf.Default_mode);
  webui.clickInt("Setpoint_min", Conf.Setpoint_min);
  webui.clickInt("Setpoint_max", Conf.Setpoint_max);
  webui.clickInt("Heater_temp_safety_limit", Conf.Heater_temp_safety_limit);

  if (p.click("Sensor_multiplier"))
  {
    Conf.Sensor_multiplier = webui.getFloat("Sensor_multiplier");
  }

  // gains
  webui.clickFloat("Kp_default", Conf.Kp_default);
  webui.clickFloat("Ki_default", Conf.Ki_default);
  webui.clickFloat("Kd_default", Conf.Kd_default);
  webui.clickFloat("Kp_conservative", Conf.Kp_conservative);
  webui.clickFloat("Ki_conservative", Conf.Ki_conservative);
  webui.clickFloat("Kd_conservative", Conf.Kd_conservative);
  webui.clickFloat("Kp_aggressive", Conf.Kp_aggressive);
  webui.clickFloat("Ki_aggressive", Conf.Ki_aggressive);
  webui.clickFloat("Kd_aggressive", Conf.Kd_aggressive);
  webui.clickInt("PID_adaptive_threshold", Conf.PID_adaptive_threshold);

  if (p.click("PID_interval"))
  {
    Conf.PID_interval = webui.getInt("PID_interval") * 1000;
  }

  // misc
  webui.clickBool("Log_enabled", Conf.Log_enabled);
  webui.clickBool("Display_heartbeat", Conf.Display_heartbeat);
  webui.clickInt("Display_brightness", Conf.Display_brightness);
  webui.clickInt("AP_radio_channel", Conf.AP_radio_channel);
  webui.clickBool("AP_hidden", Conf.AP_hidden);

  if (p.click("save_settings"))
  {
    save_config();
    ESP.reset();
  }

  if (p.click("Admin_defaults"))
  {
    load_defaults();
  }

  if (p.click("Admin_defaults_save"))
  {
    load_defaults();
    save_config();
    ESP.reset();
  }

  if (p.click("Admin_reboot"))
  {
    ESP.reset();
  }
}

//  ######  ######## ######## ##     ## ########
// ##    ## ##          ##    ##     ## ##     ##
// ##       ##          ##    ##     ## ##     ##
//  ######  ######      ##    ##     ## ########
//       ## ##          ##    ##     ## ##
// ##    ## ##          ##    ##     ## ##
//  ######  ########    ##     #######  ##

void setup()
{

  Serial.begin(115200);
  Serial.println(); // LF after all the gibberish
  Serial.printf("%s v%i.%i.%i\n\n", Conf.app_id, Conf.ver_major, Conf.ver_minor, Conf.ver_patch);

  // encoder
  Serial.println("\nInitialize display");
  init_display();

  // encoder
  Serial.println("\nConfigure encoder");
  pinMode(ENC_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), encoder_int, CHANGE);

  Serial.println("\nSet default configuration");
  load_defaults();

  Serial.print("\nInitialize LittleFS...");
  if (LittleFS.begin())
  {
    Serial.println(" Done!");
  }
  else
  {
    Serial.println(" Failed.");
  }

  // reset settings if encoder button was hold during startup
  if (!digitalRead(ENC_BUTTON_PIN))
  {
    load_defaults();

    if (save_config())
    {
      Serial.println("Configuraton was set to default.");
      max7219.DisplayText("CFG RST", 0);
      delay(5000);
      ESP.reset();
    }
    else
    {
      Serial.println("Configuraton was NOT set to default.");
      max7219.DisplayText("CFG ERR", 0);
      delay(5000);
    }
  }

  // load configuration file
  Serial.println("\nLoading configuration file");
  if (!load_config())
  {
    Serial.println("Loading configuration file FAILED");
  }

  // transfer settings from config to the working variables
  set_encoder_value(Conf.Setpoint_default); // set default encoder position to default setpoint value
  set_mode(Conf.Default_mode);

  // WIFI
  Serial.println("\nConfigure WiFi...");
  wifi_set_opmode(STATIONAP_MODE);
  wifi_set_channel(Conf.AP_radio_channel);

  // if (Conf.AP_mac_address[0])
  // {
  //   // If the mac address does not start from zero, set it for the access point
  //   wifi_set_macaddr(SOFTAP_IF, Conf.AP_mac_address);
  // }

  // create an SSID from the application ID and mac address
  uint8_t ap_mac[6];
  char radio_ssid[32];
  WiFi.softAPmacAddress(ap_mac);
  sprintf(radio_ssid, "%s_%02X%02X%02X", Conf.app_id, ap_mac[3], ap_mac[4], ap_mac[5]);

  // create a password based on the mac address
  sprintf(AP_password, "%02X%02X%02X%02X", ap_mac[5], ap_mac[0], ap_mac[2], ap_mac[0]);

  // set IP and start Soft-AP
  WiFi.softAPConfig(IPAddress(Conf.AP_ip_addr), IPAddress(Conf.AP_ip_gw), IPAddress(Conf.AP_ip_mask));
  WiFi.mode(WIFI_AP_STA);
  if (WiFi.softAP(radio_ssid, AP_password, Conf.AP_radio_channel, Conf.AP_hidden))
  {
    Serial.printf("Soft-AP: SSID=%s, KEY=%s, channel=%i, IP=", radio_ssid, AP_password, WiFi.channel());
    Serial.println(WiFi.softAPIP());
  }
  else
  {
    Serial.println("Failed to initialize Soft-AP.");
  }

  Serial.println("\nConfigure web interface");
  webui.enableOTA("ota", "ota"); // allow OTA updated with login "ota" and password "ota"
  webui.setBufferSize(2000); // set page buffer size, bytes (default is 1000)
  webui.setFS(&LittleFS);    // provide the LittleFS file system to the web server
  webui.downloadAuto(true);
  webui.uploadAuto(true);
  webui.attachBuild(webui_build);
  webui.attach(webui_action);
  webui.start();

  // heater
  Serial.println("\nConfigure heater");
  analogWriteRange(HEATER_PWM_MAX);
  analogWriteFreq(HEATER_PWM_FREQ);
  pinMode(HEATER_SSR_PIN, OUTPUT);
  set_heater(0);

  // PID
  Serial.println("\nConfigure PID");
  update_sensor();
  pid_controller.SetTunings(Conf.Kp_default, Conf.Ki_default, Conf.Kd_default);
  pid_controller.SetMode(QuickPID::Control::automatic);
  pid_controller.Initialize();

  Serial.println("Setup is done.");
}

// ##        #######   #######  ########
// ##       ##     ## ##     ## ##     ##
// ##       ##     ## ##     ## ##     ##
// ##       ##     ## ##     ## ########
// ##       ##     ## ##     ## ##
// ##       ##     ## ##     ## ##
// ########  #######   #######  ##

void loop()
{

  webui.tick();
  encoder_button.update();

  update_sensor();
  update_display(Encoder_value, floor(Sensor_corrected));
  update_plot_values();

  // switch device operating modes if button was pressed
  if (encoder_button.isSingleClick())
  {
    next_mode();
  }

  // duty loop
  if (Duty_loop_time <= millis())
  {
    Duty_loop_time = millis() + Conf.PID_interval;

    switch (Mode_current)
    {
    case MODE_BASIC:
      pid_default(Encoder_value, Sensor_corrected);
      break;

    case MODE_ADAPTIVE:
      pid_adaptive(Encoder_value, Sensor_corrected);
      break;

    case MODE_AUTOTUNER:
      autotuner_tick(Atune_setpoint, Sensor_corrected);
      break;

    case MODE_PROGRAM1:
      program_tick(1, Sensor_corrected);
      break;

    case MODE_PASSWORD:
      set_heater(0);
      break;

    default:
    case MODE_ERROR:
      set_heater(0);
      break;
    }

    // add values to the "history" page buffer
    update_plot_values();
    GPaddInt(Plot_setpoint, History_arr[0], HISTORY_SIZE);
    GPaddInt(Plot_sensor, History_arr[1], HISTORY_SIZE);
    GPaddInt(Plot_pid, History_arr[2], HISTORY_SIZE);
  }
}
