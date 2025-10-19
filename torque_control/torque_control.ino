// 闭环转矩控制（FOC）示例 - 位置传感器 AS5047P (ABI)
// 参考 basic_foc_driver.ino 的引脚和供电配置
// 需要安装 SimpleFOC 库: https://github.com/simplefoc/Arduino-FOC

#include <SimpleFOC.h>

// ===== 电机与驱动 =====
// 根据你的电机极对数设置（示例为2212类电机，极对数常见为6）
BLDCMotor motor = BLDCMotor(6);
// 与 basic_foc_driver.ino 保持一致：A=9, B=6, C=5
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 6, 5);

// ===== 编码器：AS5047P (ABI) =====
// 根据你的接线修改以下引脚与CPR
const int ENC_A = 10;        // A通道 - D10
const int ENC_B = 11;        // B通道 - D11
const int ENC_I = 12;        // 索引通道 - D12
const int ENC_CPR = 4096;
const int ENC_PPR = 1000;    // AS5047P 默认十进制模式：1000 PPR (4000 steps)

Encoder encoder = Encoder(ENC_A, ENC_B, ENC_PPR, ENC_I);

// ISR 包装函数
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
void doI() { encoder.handleIndex(); }

// UNO(ATmega328P) 上 D8-D13 使用 PCINT 兼容方案
#if defined(__AVR_ATmega328P__)
volatile uint8_t prevPINB = 0;
void setupPcintForEncoder() {
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(ENC_I, INPUT);
  prevPINB = PINB;
  // 使能 Port B 的 Pin Change Interrupt
  PCICR |= (1 << PCIE0);
  // D8-D13 对应 PCINT0-5，位号等于 pin-8
  PCMSK0 |= (1 << (ENC_A - 8)) | (1 << (ENC_B - 8)) | (1 << (ENC_I - 8));
}
ISR(PCINT0_vect) {
  uint8_t cur = PINB;
  uint8_t changed = prevPINB ^ cur;
  prevPINB = cur;
  if (changed & (1 << (ENC_A - 8))) encoder.handleA();
  if (changed & (1 << (ENC_B - 8))) encoder.handleB();
  if (changed & (1 << (ENC_I - 8))) encoder.handleIndex();
}
#endif

// 目标：q轴转矩（电压模式时单位约为[V]）
float target_torque = 0.0f;

// 命令接口
Commander command = Commander(Serial);
void onTarget(char* cmd) { command.scalar(&target_torque, cmd); }
void onLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[BOOT] torque_control starting");
  SimpleFOCDebug::enable(&Serial);
  Serial.print("ENC pins A="); Serial.print(ENC_A);
  Serial.print(" B="); Serial.print(ENC_B);
  Serial.print(" I="); Serial.println(ENC_I);
  Serial.print("PPR="); Serial.print(ENC_PPR);
  Serial.print(" CPR="); Serial.println(ENC_PPR * 4);

  // ===== 驱动器供电与电压限制（与 basic_foc_driver 保持一致） =====
  driver.voltage_power_supply = 11.1;  // 3S 电池示例
  driver.voltage_limit = 3;            // 低阻电机，限制电压保护

  // 3PWM驱动的 INH 置高（如 basic_foc_driver 所示）
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(8, HIGH);
  digitalWrite(7, HIGH);
  digitalWrite(4, HIGH);

  Serial.println("初始化驱动器(3PWM)...");
  if(!driver.init()){
    Serial.println("驱动器初始化失败!");
    return;
  }
  motor.linkDriver(&driver);

  // ===== 初始化编码器（AS5047P ABI） =====
  encoder.pullup = Pullup::USE_EXTERN;  // 如需内部上拉：Pullup::INPUT_PULLUP
  encoder.init();

  // UNO 使用 PCINT 方案，其他板或D2/D3用外部中断
  #if defined(__AVR_ATmega328P__)
    setupPcintForEncoder();
  #else
    encoder.enableInterrupts(doA, doB, doI);
  #endif

  // 位置传感器接入电机
  motor.linkSensor(&encoder);

  // ===== 选择闭环转矩控制 =====
  motor.controller = MotionControlType::torque;             // 闭环运动控制选择“转矩”
  motor.torque_controller = TorqueControlType::voltage;     // 无电流检测时用电压模式

  // 电机安全限制
  motor.voltage_limit = 3;   // 与driver.voltage_limit一致即可
  motor.velocity_limit = 20; // 启动/调试更稳妥

  // 初始化电机与FOC
  if(!motor.init()){
    Serial.println("电机初始化失败!");
    return;
  }
  Serial.println("开始FOC初始化与传感器对齐...");
  motor.initFOC();
  Serial.println("FOC初始化完成!");

  // 命令行交互：设置目标转矩与限制
  command.add('T', onTarget, "目标转矩(电压)");
  command.add('L', onLimit,  "电压限制");

  Serial.println("闭环转矩控制就绪!");
  Serial.println("通过串口发送: T 1.5  设置目标转矩为 1.5[V]");
  Serial.println("通过串口发送: L 3    设置电压限制为 3[V]");
}

// 调试：周期性打印 ABI 状态与角度/计数
void printABIStatus() {
  static uint32_t last_ms = 0;
  const uint16_t period_ms = 100; // 10Hz 打印
  uint32_t now = millis();
  if (now - last_ms < period_ms) return;
  last_ms = now;

  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);
  int iz = digitalRead(ENC_I);

  float angle_rad = encoder.getAngle();
  float angle_deg = angle_rad * 180.0f / PI;
  float vel_rad_s = encoder.getVelocity();

  long cpr = (long)ENC_PPR * ((encoder.quadrature == Quadrature::ON) ? 4 : 1);
  long count = (long)((angle_rad / (2.0f * PI)) * cpr);
  // 归一到 [0, cpr)
  long count_mod = count % cpr;
  if (count_mod < 0) count_mod += cpr;

  Serial.print("ABI A:"); Serial.print(a);
  Serial.print(" B:"); Serial.print(b);
  Serial.print(" I:"); Serial.print(iz);
  Serial.print(" | angle(deg): "); Serial.print(angle_deg, 2);
  Serial.print(" vel(rad/s): "); Serial.print(vel_rad_s, 2);
  Serial.print(" | count: "); Serial.print(count_mod);
  Serial.print(" CPR: "); Serial.print(cpr);
  Serial.print(" quadrature:"); Serial.print(encoder.quadrature == Quadrature::ON ? "ON" : "OFF");
  Serial.println();
}

void loop() {
  // 闭环：位置传感器 -> FOC估算
  motor.loopFOC();

  // 施加目标转矩（电压模式时表示q轴电压幅值）
  motor.move(target_torque);

  // 命令行交互
  command.run();

  // 调试打印 ABI 状态
  printABIStatus();
}
