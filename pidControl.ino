// Ball PID control Arduino code
#include <Servo.h>

/////////////////////////////
// Configurable paramet+ers //
/////////////////////////////

//noise_filter 제거위한 코드 가져옴
#define _INTERVAL_DIST 30  // DELAY_MICROS * samples_num^2 의 값이 최종 거리측정 인터벌. 넉넉하게 30ms 
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값)
#define EMA_ALPHA 0.35     // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값
float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플 3개로 충분




// Arduino pin assignment
#define PIN_LED 9                            //  LED를 아두이노의 GPIO 9번 핀에 연결
#define PIN_SERVO 10    // 서보모터를 아두이노의 10번 핀에 연결
#define PIN_IR A0     // IR센서를 아두이노의 A0 핀에 연결

// Framework setting
#define _DIST_TARGET 255  //목표로 하는 탁구공 중심 위치까지 거리255mm로 고정

// 측정범위 제한
#define _DIST_MIN 50                       // 최소 측정 거리 50mm로 고정 
#define _DIST_MAX 410   // 측정 거리의 최댓값를 410mm로 설정

// Distance sensor
//#define _DIST_ALPHA 0.0  // ema 필터의 alpha 값을 0.0으로 설정

// Servo range
#define _DUTY_MIN 700     //서보의 가동 최소 각도
#define _DUTY_NEU 1500        //servo neutral position (90 degree)
#define _DUTY_MAX 2100                // 서보의 최대 가동 각도
#define _POS_START (_DUTY_MIN + 100)


// Servo speed control
#define _SERVO_ANGLE 100   // 서보의 각도(100º) 
#define _SERVO_SPEED 600             //  서보 속도를 30으로 설정 // d제어할때 2000 해야 풀 속도

// Event periods
#define _INTERVAL_SERVO 20 // 서보를 20ms마다 조작하기
#define _INTERVAL_SERIAL 100  //  시리얼 0.1초 마다 업데이트

// PID parameters
#define _KP 2     
#define _KI 0.006
#define _KD 80
#define _ITERM_MAX 78
// 255일때 42 stable, speed 600 
// 400일때 28 stable, speed 600 
// underdamped: KP 1.2,  KD 42
// over damped: KP: 1.2, KD: 102
// critically damped: KP:1.2, KD:56
// PID 제어 KP:2, KI: 0.0061, KD: 80.2


//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;  //  Servo를 제어할 Object를 생성
// Distance sensor
float dist_target; // location to send the ball 
float dist_raw, dist_ema;    // 실제 거리측정값과 ema필터를 적용한 값을 저장할 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; //  거리, 서보, 시리얼에 대한 마지막 샘플링 시간을 나타내는 변수
bool event_dist, event_servo, event_serial; // 거리센서, 서보, 시리얼 모니터의 발생을 지시하는 변수


// Servo speed control
int duty_chg_per_interval;    // 주기동안 duty 변화량 변수
int duty_target, duty_curr;    // 보의 목표위치와 서보에 실제로 입력할 위치
int duty_neutral;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; //  비례 제어를 위한 전에 측정한 오차, 새로 측정한 오차 값, 비례항, 적분항, 미분항 변수


// IR sensor calibraton
const float coE[] = {0.0000055, -0.0058625, 2.5600221, -109.4361340};

float x;

unsigned long time_curr;
unsigned long time_end;




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
// initialize GPIO pins for LED and attach servo 

  pinMode(PIN_LED, OUTPUT);  // LED 핀 설정
  myservo.attach(PIN_SERVO);  // Servo 핀 설정
// initialize global variables
  pterm = dterm = iterm = 0;
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0; //  샘플링 시각 기록 변수 초기화
  event_dist = event_servo = event_serial = false;  //  이벤트 bool값 초기화
  dist_target = _DIST_TARGET; // 목표지점 변수 초기화

  duty_target = duty_curr = _POS_START;
  
// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); //서보를 중간으로 이동
// initialize serial port
    Serial.begin(57600);                          //  57600 보드레이트로 아두이노와 통신
// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (float(_SERVO_SPEED) /  float(_SERVO_ANGLE)) * (float(_INTERVAL_SERVO) / 1000.0);   // 서보의 각속도를 원하는 Angle로 나누어 interval을 설정
  
  Serial.print("duty_chg_per_interval");
  Serial.println(duty_chg_per_interval);

}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
void loop() {
/////////////////////
// Event generator // 설정된 주기마다 이벤트 생성
/////////////////////

duty_neutral = _DUTY_NEU;

time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
time_end = millis();
////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false;
  // get a distance reading from the distance sensor

      // 거리센서 필터 적용 값
      x = ir_distance_filtered();
      dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
      
      
  // PID control logic
    error_curr = _DIST_TARGET - dist_raw; // [3158] 목표값 에서 현재값을 뺀 값이 오차값
    pterm = _KP*error_curr;
    iterm += _KI*error_curr;

    if(abs(iterm) > _ITERM_MAX) iterm = 0;
    if(iterm > _ITERM_MAX) iterm = _ITERM_MAX;
    if(iterm < - _ITERM_MAX) iterm = - _ITERM_MAX;
    
    dterm = _KD * (error_curr - error_prev);
    control = pterm + dterm + iterm;
 
   //duty_target = f(duty_neutral, control
     duty_target = duty_neutral + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;  // duty_target < _DUTY_MIN 일 때 duty_target 를 _DUTY_MIN 로 고정
    }
    else if (duty_target > _DUTY_MAX) {
       duty_target = _DUTY_MAX; // duty_target > _DUTY_MAX 일 때 duty_target 를 _DUTY_MAX 로 고정
    }     //  (_DUTY_MIN, _DUTY_MAX) 로 서보의 가동범위를 고정하기 위한 최소한의 안전장치

 // update error_prev
  error_prev = error_curr;
 
 }
  
  if(event_servo) {
    event_servo = false; // [3153] servo EventHandler Ticket -> false
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }  
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }       // 서보가 현재위치에서 목표위치에 도달할 때까지 duty_chg_per_interval값 마다 움직임(duty_curr에 duty_chg_per_interval값 더하고 빼줌)
    
    // update servo position
    myservo.writeMicroseconds(duty_curr);   // 위에서 바뀐 현재위치 값을 갱신
  }

  
  
 if(event_serial) {
event_serial = false;
Serial.print("IR:");
Serial.print(dist_raw);
Serial.print(",T:");
Serial.print(dist_target);
Serial.print(",P:");
Serial.print(map(pterm,-1000,1000,510,610));
Serial.print(",D:");
Serial.print(map(dterm,-1000,1000,510,610));
Serial.print(",I:");
//Serial.print(iterm);

Serial.print(map(iterm,-1000,1000,510,610));
Serial.print(",DTT:");
Serial.print(map(duty_target,1000,2000,410,510));
Serial.print(",DTC:");
Serial.print(map(duty_curr,1000,2000,410,510));
Serial.println(",-G:245,+G:265,m:0,M:800");
}

  
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;       //   적외선 센서를 통해 거리를 return
}

///////////////////// noise_filter 코드 
float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
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

float ir_distance_filtered(void) { // return value unit: mm

  // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종값 생성
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
