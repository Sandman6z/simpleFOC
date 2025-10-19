/**
 * 基于Arduino-SimpleFOC-PowerShield的开环速度控制程序
 * 
 * 此程序用于驱动无刷电机，使用SimpleFOC库和PowerShield硬件
 * 需要安装SimpleFOC库: https://github.com/simplefoc/Arduino-FOC
 */

#include <SimpleFOC.h>

// BLDCMotor motor = BLDCMotor(极对数);
BLDCMotor motor = BLDCMotor(6);
// BLDCDriver3PWM 使用各相IN脚作为PWM: A=9, B=6, C=5
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 6, 5);

// 电流检测引脚
const int currentSenseA = A0;  // 相位A电流检测
const int currentSenseC = A1;  // 相位C电流检测
const int btn8982_isA = A2;    // BTN8982诊断A
const int btn8982_isB = A3;    // BTN8982诊断B

// 目标变量
float target_velocity = 0;

// 实例化命令接口
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {
  // 初始化串口通信
  Serial.begin(115200);
  // 启用更详细的调试输出
  SimpleFOCDebug::enable(&Serial);
  
  // 初始化电流检测引脚
  pinMode(currentSenseA, INPUT);
  pinMode(currentSenseC, INPUT);
  pinMode(btn8982_isA, INPUT);
  pinMode(btn8982_isB, INPUT);
  
  // 驱动器配置
  // 电源电压设置 [V] - 大疆2212电机通常使用11.1V (3S锂电池)
  driver.voltage_power_supply = 11.1;
  // 限制驱动器可以设置的最大直流电压
  // 大疆2212是低阻电机，从低电压开始以保护电机
  driver.voltage_limit = 3;

  // 手动拉高各相INH使能引脚（3PWM模式）
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(8, HIGH);
  digitalWrite(7, HIGH);
  digitalWrite(4, HIGH);
  
  // 添加详细的错误诊断（3PWM + INH高电平）
  Serial.println("正在初始化驱动器(3PWM)...");
  Serial.print("PWM引脚: A="); Serial.print(9);
  Serial.print(" B="); Serial.print(6);
  Serial.print(" C="); Serial.println(5);
  Serial.print("INH置高: A="); Serial.print(8);
  Serial.print(" B="); Serial.print(7);
  Serial.print(" C="); Serial.println(4);
  
  if(!driver.init()){
    Serial.println("驱动器初始化失败!");
    Serial.println("可能的原因:");
    Serial.println("1. INH未正确拉高或接线错误");
    Serial.println("2. 电源问题 - 确保电源电压稳定且足够");
    Serial.println("3. 驱动器芯片问题 - 检查BTN8982TA是否正常工作");
    Serial.println("4. Arduino引脚冲突 - 某些引脚可能与其他功能冲突");
    return;
  }
  Serial.println("驱动器初始化成功!");
  
  // 连接电机和驱动器
  motor.linkDriver(&driver);
  
  // 限制电机运动
  // 限制设置给电机的电压
  // 对于高阻电机，从很低的值开始
  // 电流 = 电压 / 电阻，所以尽量保持在1安培以下
  motor.voltage_limit = 3;   // [V]
  
  // 开环控制配置
  motor.controller = MotionControlType::velocity_openloop;
  
  // 初始化电机硬件
  if(!motor.init()){
    Serial.println("电机初始化失败!");
    return;
  }
  
  // 添加目标命令T和电压限制命令L
  command.add('T', doTarget, "目标速度");
  command.add('L', doLimit, "电压限制");
  
  Serial.println("电机就绪!");
  Serial.println("设置目标速度 [rad/s]");
  delay(1000);
}

void loop() {
  // 开环速度运动
  // 使用motor.voltage_limit和motor.velocity_limit
  // 要使电机"反向旋转"，只需设置负的target_velocity
  motor.move(target_velocity);

  // 用户通信
  command.run();
}