#include <Servo.h>
Servo myservo;
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DUTY_NEU 1450 // neutral position
#define _DUTY_MIN 1000
#define _DUTY_MAX 2000

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 30

#define _INTERVAL_DIST 20   // USS interval (unit: ms)
#define _INTERVAL_SERVO 20  // servo interval (unit: ms)
#define _INTERVAL_SERIAL 100 // serial interval (unit: ms)

#define _KP 0.9


#define DELAY_MICROS  1500
#define EMA_ALPHA 0.5

unsigned long last_sampling_time_dist, last_sampling_time_serial; // unit: ms
bool event_dist, event_serial, event_servo;
float dist_raw, dist_target;
float error_curr, control, pterm;

int duty_chg_per_interval;
int duty_target, duty_curr;

float dist_ema;
float ema_dist = 0;
float samples_num = 3;

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

int a, b;

// ================
float under_noise_filter(void){
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}
float filtered_ir_distance(void){
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
//===================================================

void setup() {
  Serial.begin(57600);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  delay(1000);
  // initialize last sampling time
  last_sampling_time_dist = last_sampling_time_serial = 0;
  event_dist = event_serial = false;
  dist_target = _DIST_TARGET;


  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN)*(_SERVO_SPEED/_SERVO_ANGLE)*
  (_INTERVAL_SERVO/1000.0);
}

void loop() {
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
  if(event_dist) {
    event_dist = false;
    dist_raw = ir_distance();
    dist_ema = filtered_ir_distance();

    error_curr = dist_target-dist_ema;
    pterm = error_curr;
    control = _KP* pterm;

    duty_target = _DUTY_NEU + control;
  }
  if(event_servo){
    event_servo = false;
    if(duty_target > duty_curr) {
        duty_curr += duty_chg_per_interval;
        if(duty_curr > duty_target) duty_curr = duty_target;
      }
    else {
        duty_curr -= duty_chg_per_interval;
        if(duty_curr < duty_target) duty_curr = duty_target;
      }
   }
  if(event_serial) {
    event_serial = false;
    if(dist_ema < 255){
      myservo.writeMicroseconds(1450 + control);
    }
    else{
      myservo.writeMicroseconds(1450 + control);
    }
    // output the read value to the serial port
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
 }
