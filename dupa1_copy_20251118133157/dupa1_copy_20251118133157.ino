#include <math.h>
#include <Servo.h>
#include <avr/io.h>
#include <util/delay.h>

#define ADC_PIN PC0
#define ADC_REF_VOLTAGE 5.0f // reference voltage AVcc (5 V for Arduino)
#define ADC_RESOLUTION 1023.0f // 10-bit ADC (0-1023)

Servo head_servo; // create servo object to control a servo
#define DEFAULT_SERVO_POS 90

int state_machine = 0;
bool touch_detected = false;
int double_touch_detected = 0; // 0 - no touch, 1 - first touch, 2 - double touch
long long unsigned int timer_cnt = 0;
#define DOUBLE_TOUCH_THRESH 0.6

float alpha = 0.2;
float beta = 0.001;
float filterred_pressure = 0.0;

float xk_1 = 0.0;
float vk_1 = 0.0;
float xk = 0.0;
float vk = 0.0;

float rk = 0.0;

float dt = 0.05;

float belly_integral = 0.0;
#define UPPER_INTEGRAL_BELLY_LIM 0.8
#define LOWER_INTEGRAL_BELLY_LIM -0.1

float last_filterred_pressure = 0.0;

#define TOUCH_COOLDOWN_TIME 4.0
float touch_cooldown = 0.0;

#define BETWEEN_ANIM_COOLDOWN_TIME 1.0
float between_anim_cooldown = BETWEEN_ANIM_COOLDOWN_TIME;
bool anim_playing = false;
int anim_cnt = 0;
float average_pressure = 0.0;
uint64_t average_cnt = 0;

// Motor1 - belly pump
// Motor2 - right ear pump
// Motor3 - left ear pump
// Motor4 - ears valve

const int E1 = 3; ///<Motor1 Speed
const int E2 = 11;///<Motor2 Speed
const int E3 = 5; ///<Motor3 Speed
const int E4 = 6; ///<Motor4 Speed

const int M1 = 4; ///<Motor1 Direction
const int M2 = 12;///<Motor2 Direction
const int M3 = 8; ///<Motor3 Direction
const int M4 = 7; ///<Motor4 Direction

void initADC()
{
  ADMUX = (1 << REFS0);               // reference voltage AVcc
  ADCSRA = (1 << ADEN) |              // turn on ADC
           (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // 128 prescaler
  _delay_ms(1);                       // ADC stabilisation
}

uint16_t readADC(uint8_t channel)
{
  ADMUX = (1 << REFS0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);              // convertion start
  while (ADCSRA & (1 << ADSC));       // wait till end
  return ADC;
}

float readADC_Voltage(uint8_t channel)
{
  // Set channel and reference
  ADMUX = (1 << REFS0) | (channel & 0x0F);

  // Start the convertion
  ADCSRA |= (1 << ADSC);
  
  // Wait for the convertion end
  while (ADCSRA & (1 << ADSC));
  
  // Read ADC value and convert to voltage
  uint16_t adc_value = ADC;
  float voltage = (adc_value * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
  
  return voltage;
}

void M1_advance(uint8_t Speed) ///<Motor1 Advance
{
  if (Speed < 50)
  {
    Speed = 0;
  }

  digitalWrite(M1,LOW);
  analogWrite(E1,Speed);
}

void M2_advance(uint8_t Speed) ///<Motor2 Advance
{
  digitalWrite(M2,HIGH);
  analogWrite(E2,Speed);
}

void M3_advance(uint8_t Speed) ///<Motor3 Advance
{
  digitalWrite(M3,LOW);
  analogWrite(E3,Speed);
}

void M4_advance(uint8_t Speed) ///<Motor4 Advance
{
  digitalWrite(M4,HIGH);
  analogWrite(E4,Speed);
}

void M1_back(uint8_t Speed) ///<Motor1 Back off
{
  digitalWrite(M1,HIGH);
  analogWrite(E1,Speed);
}

void M2_back(uint8_t Speed) ///<Motor2 Back off
{
  digitalWrite(M2,LOW);
  analogWrite(E2,Speed);
}

void M3_back(uint8_t Speed) ///<Motor3 Back off
{
  digitalWrite(M3,HIGH);
  analogWrite(E3,Speed);
}

void M4_back(uint8_t Speed) ///<Motor4 Back off
{
  digitalWrite(M4,LOW);
  analogWrite(E4,Speed);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {;} // waiting for arduino to open the com port

  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("Boobing up! Please Wait....");
  Serial.println("");

  head_servo.attach(9); // attaches the servo on pin 9 to the servo object
  head_servo.write(DEFAULT_SERVO_POS); // sets the servo position according to the scaled value

  initADC();

  //DDRB |= (1 << SERVO_PIN);
  //initTimer1();
  M1_advance(0);
  M2_advance(0);
  M3_advance(0);
  M4_advance(0);  
}


#define HEAD_ANIM_MAX_TIME 5
float head_anim_time = HEAD_ANIM_MAX_TIME;

void anim_head_no(float dt)
{
  if (head_anim_time <= HEAD_ANIM_MAX_TIME)
  {
    head_anim_time += dt;

    if (head_anim_time >= 0 && head_anim_time <= 0.5)
    {
      anim_playing = true;
      head_servo.write(DEFAULT_SERVO_POS + 40);
    }

    if (head_anim_time >= 0.5 && head_anim_time <= 1.0)
    {
      head_servo.write(DEFAULT_SERVO_POS - 40);
    }

    if (head_anim_time >= 1.0 && head_anim_time <= 1.5)
    {
      head_servo.write(DEFAULT_SERVO_POS + 40);
    }

    if (head_anim_time >= 1.5 && head_anim_time <= 2.0)
    {
      head_servo.write(DEFAULT_SERVO_POS - 40);
    }

    if(head_anim_time >= 2.0 && head_anim_time <= 2.5)
    {
      head_servo.write(DEFAULT_SERVO_POS + 40);
    }

    if (head_anim_time >= 2.5 && head_anim_time <= 3.0)
    {
      head_servo.write(DEFAULT_SERVO_POS);
    }

    if (head_anim_time >= HEAD_ANIM_MAX_TIME - 0.1)
    {
      anim_playing = false;
      between_anim_cooldown = BETWEEN_ANIM_COOLDOWN_TIME;
      head_servo.write(DEFAULT_SERVO_POS);

      if (state_machine == 1)
        state_machine = 2;
    }
  }
}


#define BOTH_EARS_ANIM_MAX_TIME 5
float both_ears_anim_time = BOTH_EARS_ANIM_MAX_TIME;

void anim_both_ears(float dt)
{
  if (both_ears_anim_time <= BOTH_EARS_ANIM_MAX_TIME)
  {
    both_ears_anim_time += dt;

    if (both_ears_anim_time >= 0 && both_ears_anim_time <= 1.5)
    {
      M3_advance(250);
      M2_back(250);

      anim_playing = true;
    }

    if (both_ears_anim_time >= 1.5 && both_ears_anim_time <= 2.0)
    {
      M3_advance(180);
      M2_back(0);
    }

    if (both_ears_anim_time >= 3.5 && both_ears_anim_time <= 4.0)
    {
      M3_advance(0);
      M2_advance(0);
      M4_back(250);
    }

    if (both_ears_anim_time >= 4.0 && both_ears_anim_time <= 5.0)
    {
      M4_back(0);
    }

    if (both_ears_anim_time >= BOTH_EARS_ANIM_MAX_TIME - 0.1)
    {
      M4_back(0);
      anim_playing = false;
      between_anim_cooldown = BETWEEN_ANIM_COOLDOWN_TIME;
    }
  }
}


#define HEAD_SCAN_ANIM_MAX_TIME 10
float head_scan_anim_time = HEAD_SCAN_ANIM_MAX_TIME;

void anim_head_scan(float dt)
{
  if (head_scan_anim_time <= HEAD_SCAN_ANIM_MAX_TIME)
  {
    head_scan_anim_time += dt;

    if (head_scan_anim_time >= 0 && head_scan_anim_time <= 1.5)
    {
      head_servo.write(DEFAULT_SERVO_POS - 80);

      anim_playing = true;
    }

    if (head_scan_anim_time >= 1.5 && head_scan_anim_time <= 6.5)
    {
      float servo_angle = (DEFAULT_SERVO_POS - 80) + ((DEFAULT_SERVO_POS + 80) * ((head_scan_anim_time - 1.5) / 6.5));
      head_servo.write(servo_angle);
    }

    if (head_scan_anim_time >= 6.5 && head_scan_anim_time <= 11.5)
    {
      float servo_angle = (DEFAULT_SERVO_POS + 80) - ((DEFAULT_SERVO_POS + 80) * ((head_scan_anim_time - 6.5) / 11.5));
      head_servo.write(servo_angle);
    }

    if (head_scan_anim_time >= HEAD_SCAN_ANIM_MAX_TIME - 0.1)
    {
      anim_playing = false;
      between_anim_cooldown = BETWEEN_ANIM_COOLDOWN_TIME;
      head_servo.write(DEFAULT_SERVO_POS);

      if (state_machine == 1)
        state_machine = 2;
    }
  }
}


#define EAR_1_ANIM_MAX_TIME 10
float ear_1_anim_time = EAR_1_ANIM_MAX_TIME;

void anim_left_ear(float dt)
{
  if (ear_1_anim_time <= EAR_1_ANIM_MAX_TIME)
  {
    ear_1_anim_time += dt;

    if (ear_1_anim_time >= 0 && ear_1_anim_time <= 1.5)
    {
      anim_playing = true;
      M3_advance(250);
    }

    if (ear_1_anim_time >= 1.5 && ear_1_anim_time <= 8.0)
    {  
      M3_advance(120);
    }

    if (ear_1_anim_time >= 8.0 && ear_1_anim_time <= 9.5)
    {  
      M3_advance(0);
    }

    if (ear_1_anim_time >= 9.5 && ear_1_anim_time <= 10.0)
    {
      M3_advance(0);
      M4_back(255);
    }

    if (ear_1_anim_time >= EAR_1_ANIM_MAX_TIME - 0.1)
    {
      M4_back(0);
      anim_playing = false;
      between_anim_cooldown = BETWEEN_ANIM_COOLDOWN_TIME;
      //head_servo.write(DEFAULT_SERVO_POS);

      if (state_machine == 3)
        state_machine = 0;
    }
  }
}


#define EAR_2_ANIM_MAX_TIME 10
float ear_2_anim_time = EAR_2_ANIM_MAX_TIME;

void anim_right_ear(float dt)
{
  if (ear_2_anim_time <= EAR_2_ANIM_MAX_TIME)
  {
    ear_2_anim_time += dt;

    if (ear_2_anim_time >= 0 && ear_2_anim_time <= 1.5)
    {
      anim_playing = true;
      M2_back(250);
    }

    if(ear_2_anim_time >= 1.5 && ear_2_anim_time <= 8.0)
    {  
      M2_back(120);
    }

    if (ear_2_anim_time >= 8.0 && ear_2_anim_time <= 9.5)
    {
      M2_back(0);
    }

    if (ear_2_anim_time >= 9.5 && ear_2_anim_time <= 10.0)
    {
      M2_back(0);
      M4_back(255);
    }

    if (ear_2_anim_time >= EAR_2_ANIM_MAX_TIME - 0.1)
    {
      M4_back(0);
      anim_playing = false;
      between_anim_cooldown = BETWEEN_ANIM_COOLDOWN_TIME;
      // head_servo.write(DEFAULT_SERVO_POS);

      if (state_machine == 2)
        state_machine = 3;
    }
  }
}


#define SAD_ANIM_MAX_TIME 20
float sad_anim_time = SAD_ANIM_MAX_TIME;

void anim_sad_ears(float dt)
{
  if (sad_anim_time <= SAD_ANIM_MAX_TIME)
  {
    sad_anim_time += dt;

    if (sad_anim_time >= 0 && sad_anim_time <= 1.5)
    {
      anim_playing = true;
      M3_advance(250);
      M2_back(250);
      touch_cooldown = 0.0;
    }

    if (sad_anim_time >= 1.5 && sad_anim_time <= 18)
    {
      M3_advance(180);
      M2_back(120);
    }

    // waiting for touch
    if (sad_anim_time >= 2.0 && sad_anim_time <= 15 && touch_detected) // touch_cooldown >= TOUCH_COOLDOWN_TIME - 0.2)
    {
      M4_back(255);
      sad_anim_time = 17.5;

      if (state_machine == 0)
        state_machine = 1;
      Serial.println("Playing phase 2, ears up!");
    }

    if (sad_anim_time >= 18 && sad_anim_time <= SAD_ANIM_MAX_TIME)
    {
      M3_advance(0);
      M2_back(0);
      M4_back(255);
    }

    if (sad_anim_time >= SAD_ANIM_MAX_TIME - 0.1)
    {
      M4_back(0);
      M3_advance(0);
      M2_back(0);
      anim_playing = false;
      between_anim_cooldown = BETWEEN_ANIM_COOLDOWN_TIME;
    }
  }
}


void loop()
{
  // Measure pressure
  float adc_voltage_0 = readADC_Voltage(0);
  float xm = ((float)adc_voltage_0 - 0.2) / 0.045f;

  xk = xk_1 + (vk_1 * dt);
  vk = vk_1;

  rk = xm - xk;
  xk += alpha * rk;

  vk += (beta * rk ) / dt;

  xk_1 = xk;
  vk_1 = vk;

  filterred_pressure = xk;

  // Belly inflating
  float belly_error = (3.8 - filterred_pressure);
  belly_integral += belly_error * dt;

  if (belly_integral >= UPPER_INTEGRAL_BELLY_LIM)
    belly_integral = UPPER_INTEGRAL_BELLY_LIM;
  
  if (belly_integral <= LOWER_INTEGRAL_BELLY_LIM)
    belly_integral = LOWER_INTEGRAL_BELLY_LIM;

  int motor_belly = belly_error * 50.0 + belly_integral * 100.0;

  if (motor_belly <= 0)
    motor_belly = 0;
  
  if (motor_belly >= 255)
    motor_belly = 255;

  average_cnt++;
  average_pressure = ((average_pressure * (average_cnt-1)) + filterred_pressure) / average_cnt;

  M1_advance((int)(motor_belly));

  // Serial.print("Pressure: ");
  // Serial.println(filterred_pressure);
  // Serial.print("AVG pressure: ");
  // Serial.println(average_pressure);
  //   Serial.print("Kp: ");
  // Serial.println(belly_error * 150);
  //   Serial.print("Ki: ");
  // Serial.println(belly_integral * 150);

  // Serial.print("Motor: ");
  // Serial.println((int)(motor_belly));

  // Touch detection (dotknij grzyba)
  touch_detected = false;
  if (double_touch_detected == 2)
    double_touch_detected = 0;

  if (filterred_pressure - average_pressure > 0.2 && motor_belly <= 50 && filterred_pressure > 4.0)
  {
    if (touch_cooldown <= 0.0)
    {
      touch_detected = true;

      Serial.print("Touch detected: ");
      Serial.println(filterred_pressure);

      touch_cooldown = TOUCH_COOLDOWN_TIME;
    }

    if (timer_cnt * dt >= DOUBLE_TOUCH_THRESH)
    {
      if (double_touch_detected == 1)
      {
        double_touch_detected = 2;
        Serial.print("DOUBLE TOUCH DETECTED!\n\n");
      }
      else
      {
        double_touch_detected = 1;
      }

      timer_cnt = 0;
    }
  }
  else if (timer_cnt * dt >= DOUBLE_TOUCH_THRESH && double_touch_detected == 1)
  {
    double_touch_detected = 0;
    timer_cnt = 0;
  }

  Serial.print("state: ");
  Serial.println(state_machine);

  // State machine
  if (between_anim_cooldown <= 0 && anim_playing == false)
  {
    switch (state_machine)
    {
      case 0:
        if (sad_anim_time >= SAD_ANIM_MAX_TIME - 0.2)
        {
          sad_anim_time = 0.0;
          Serial.println("Playing animation, sad ears!");
        }
        break;
      case 1:
        if (head_anim_time >= HEAD_ANIM_MAX_TIME - 0.2)
        {
          head_anim_time = 0.0;
          Serial.println("Playing animation, head no!");
        }
        break;
      case 2:
        if (ear_2_anim_time >= EAR_2_ANIM_MAX_TIME - 0.2)
        {
          ear_2_anim_time = 0.0;
          Serial.println("Playing animation, right ear down!");
        }
        break;
      case 3:
        if (ear_1_anim_time >= EAR_1_ANIM_MAX_TIME - 0.2)
        {
          ear_1_anim_time = 0.0;
          Serial.println("Playing animation, left ear down!");
        }
        break;
    }
  }

  // // tucz detekszyn
  // if(filterred_pressure - average_pressure > 0.2 && touch_cooldown <= 0.0 && motor_belly <= 50 ) {
  //   Serial.print("Touch detected: ");
  //   Serial.println(filterred_pressure);

  //   touch_cooldown = TOUCH_COOLDOWN_TIME;

  //   if(between_anim_cooldown <= 0 && anim_playing == false) {


  //     if(head_scan_anim_time >= HEAD_SCAN_ANIM_MAX_TIME - 0.2  && anim_cnt == 0){
  //       head_scan_anim_time = 0.0;
  //       Serial.println("Playing animation, scan head!");
  //     }

  //     if(sad_anim_time >= SAD_ANIM_MAX_TIME - 0.2  && anim_cnt == 1){
  //       sad_anim_time = 0.0;
  //       Serial.println("Playing animation, sad ears!");
  //     }

  //     if(head_anim_time >= HEAD_ANIM_MAX_TIME - 0.2  && anim_cnt == 2){
  //       head_anim_time = 0.0;
  //       Serial.println("Playing animation, head no!");
  //     }

  //     if(both_ears_anim_time >= BOTH_EARS_ANIM_MAX_TIME - 0.2  && anim_cnt == 3){
  //       both_ears_anim_time = 0.0;
  //       Serial.println("Playing animation, both ears down!");
  //     }

  //     if(ear_2_anim_time >= EAR_2_ANIM_MAX_TIME - 0.2 && anim_cnt == 4) {
  //       ear_2_anim_time = 0.0;
  //       Serial.println("Playing animation, right ear down!");
  //     }

  //     anim_cnt ++;
  //     if(anim_cnt >= 5)
  //       anim_cnt = 0;

  //   }
  // }

  // Time passes by
  if (touch_cooldown >= 0.0)
  {
    touch_cooldown -= dt;
  }

  if (between_anim_cooldown >= 0.0)
  {
    between_anim_cooldown -= dt;
  }

  // ANIMS
  anim_both_ears(dt);
  anim_right_ear(dt);
  anim_left_ear(dt);
  anim_head_no(dt);
  anim_head_scan(dt);
  anim_sad_ears(dt);

  timer_cnt++;

  // Wait a bit
  delay(dt * 1000);
}
