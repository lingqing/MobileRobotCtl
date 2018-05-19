
/***
 * 移动小车控制程序
 * @Description: 从无线接收模块接收PWM信号，然后转变为两个左右轮子的运动控制信号
 *               当前接收两路PWM信号，分别为前进后退信号speeds和方向信号dirValue
 *               最后转换为左右轮子控制信号，主要包括方向信号和速度大小信号speeds1，speeds2
 * @date: 2018-5-13
 * @Author Andy
 */
// 接线端口定义
#define READ_PIN  A0  // 前进后退信号接收
#define DIR_PIN   A1  // 左右方向

// 两个驱动电机控制信号
#define VR_OUT1    10
#define STOP_OUT1   A2 
#define DIR_OUT1   A3

#define VR_OUT2    11
#define STOP_OUT2   A4 
#define DIR_OUT2   A5

// local varity
unsigned long duration = 0;
unsigned long dur_dir = 0;
float speeds = 0;
float speeds1 = 0, speeds2 = 0;
float dirValue = 0;
//int lastdir1 = 0, lastdir2 = 0, thisdir1 = 0, thisdir2 = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // OUTPUT INPUT PIN Set
  pinMode(READ_PIN, INPUT);
  pinMode(DIR_PIN, INPUT);
  
  pinMode(VR_OUT1, OUTPUT);
  pinMode(STOP_OUT1, OUTPUT);
  pinMode(DIR_OUT1, OUTPUT);
  
  pinMode(VR_OUT2, OUTPUT);
  pinMode(STOP_OUT2, OUTPUT);
  pinMode(DIR_OUT2, OUTPUT);
  // Set Stop Pin High to low
  digitalWrite(STOP_OUT1, HIGH);
  digitalWrite(STOP_OUT2, HIGH);
  delay(100);
  digitalWrite(STOP_OUT1, LOW);
  digitalWrite(STOP_OUT2, LOW);
}

/**
 * 设置安全范围，将接收到的信号转换到安全范围区间
 * @Para: value：输入信号
 *        T ：需要控制到的最到阈值
 *        offset：偏差，当信号小雨offset时，信号忽略，置为0
 */
float getSafeValue(float value, float T, float offset = 0.1){
  T = abs(T);
  offset = abs(offset);
  if(abs(value) < offset){
    return 0.0f;
  }
  // add offset
  if(value > 0){
    value -= offset;
    if(value > T) value = T;
  }
  else{
    value += offset;
    if(value < -T) value = -T;
  }
//  // set max
//  if(abs(value)>1){
//    if(value > 0) return 1.0;
//    else return -1.0;
//  }
  return value;
}

/***
 * filter
 * 对参数进行滤波
 * output = 0.8 * (0.7* (sum2 /5 - sum1 /5) + sum2/5) + 0.2 * valuein
 *        = 0.272 * sum2 - 0.112 * sum1 + 0.2 * valuein
 * 为了使滤波函数不至于超调，可调端采样周期，delay 10ms 为较好
 */
float speedsFilter(float valueIn){
  // first init
  static float buff[10] = {valueIn};
  static float sum1 = 5* valueIn;
  static float sum2 = 5 * valueIn;
  
  // claculate result
  float ret = (0.272 * sum2 - 0.112 * sum1 + 0.2 * valueIn);
  // 重新计算 sum1, sum2
  sum1 = sum1 - buff[0] + buff[5];
  sum2 = sum2 - buff[5] + valueIn;
  // 重新排序
  for(int i = 0; i < 9; i ++){
    buff[i] = buff[i+1];
  }
  buff[9] = valueIn;
  
  return ret;
}

float dirFilter(float valueIn){
  // first init
  static float buff[10] = {valueIn};
  static float sum1 = 5* valueIn;
  static float sum2 = 5 * valueIn;
  
  // claculate result
  float ret = (0.272 * sum2 - 0.112 * sum1 + 0.2 * valueIn);
  // 重新计算 sum1, sum2
  sum1 = sum1 - buff[0] + buff[5];
  sum2 = sum2 - buff[5] + valueIn;
  // 重新排序
  for(int i = 0; i < 9; i ++){
    buff[i] = buff[i+1];
  }
  buff[9] = valueIn;
  
  return ret;
}
/**
 * 
 */
void loop() {
  // put your main code here, to run repeatedly:
  duration = pulseIn(READ_PIN, HIGH);
  dur_dir = pulseIn(DIR_PIN, HIGH);

  if(duration == 0) duration = 1500;
  if(dur_dir == 0) dur_dir = 1500;
 
  delay(10);
  speeds = (duration-1500.0f)/500.0f;
  dirValue = (dur_dir -1500.0f)/500.0f;

  if(abs(speeds) > 1.0f) return;    // 剔除异常信号，测试发现，当小车在实际行走时，由于外部受力会导致电磁场发生变化
  if(abs(dirValue) > 1.0f) return;  // 无线信号会接收到一些异常信号，这些值通常绝对值回大于1.0

  Serial.print(speeds);
  Serial.print(";");
  Serial.print(dirValue );
  Serial.print("--\t");
  speeds = getSafeValue(speeds, 1.5);
  dirValue = getSafeValue(dirValue, 1.5);

  speeds = speedsFilter(speeds);
  dirValue = dirFilter(dirValue);
//  Serial.print(speeds);
//  Serial.print(";");
//  Serial.println(dirValue );
  if(speeds >=0){             // 前进和后退时方向对左右轮的速度调整是相反的
    speeds1 = speeds - 0.5 * dirValue;
    speeds2 = speeds + 0.5 * dirValue;
  }else{
    speeds1 = speeds + 0.5 * dirValue;
    speeds2 = speeds - 0.5 * dirValue;
  }

  if(speeds1 > 1.0f) speeds1 = 1.0f;
  if(speeds1 < -1.0f) speeds1 = -1.0f;
  if(speeds2 > 1.0f) speeds2 = 1.0f;
  if(speeds2 < -1.0f) speeds2 = -1.0f;
  // multi times, max is 250/125
  speeds1 *= 100;       // 设置最大值，最大应该为 250
  speeds2 *= 100;
//
  if(abs(speeds1) <= 0.05) digitalWrite(STOP_OUT1, HIGH);
  else{
    digitalWrite(STOP_OUT1, LOW);
    if(speeds1 > 0) digitalWrite(DIR_OUT1, LOW);
    else digitalWrite(DIR_OUT1, HIGH);
  }
  if(abs(speeds2) <= 0.05) digitalWrite(STOP_OUT2, HIGH);
  else{
    digitalWrite(STOP_OUT2, LOW);
    if(speeds2 > 0) digitalWrite(DIR_OUT2, HIGH);
    else digitalWrite(DIR_OUT2, LOW);
  }

  Serial.print(speeds1);
  Serial.print(";");
  Serial.println(speeds2 );

  analogWrite(VR_OUT1, int(abs(speeds1)));
  analogWrite(VR_OUT2, int(abs(speeds2)));

}
