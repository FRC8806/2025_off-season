package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.ConsSwerve;

public class SwerveModule { 
  // ==== 成員變數 ====
  private TalonFX m_throttle;                 // 推進(行走)馬達
  private TalonFX m_rotor;                     // 轉向馬達
  private CANcoder m_encoder;                  // 轉向外接編碼器(CANcoder)
  private VelocityVoltage m_velocitySetter;   // 速度閉環控制命令（行走）
  private PositionVoltage m_angleSetter;      // 位置閉環控制命令（轉向）
  public double angle;                        // 除錯/觀察用途（未強制使用）

  // === 建構子：依模組參數初始化 ===
  public SwerveModule(ConsSwerve.Modules module) {
    m_throttle = new TalonFX(module.throttle_id);        // 依常數建立行走馬達
    m_rotor = new TalonFX(module.rotor_id);              // 依常數建立轉向馬達
    m_encoder = new CANcoder(module.encoder_id);         // 依常數建立外接 CANcoder

    m_velocitySetter = new VelocityVoltage(0);           // 行走速度閉環命令物件
    m_angleSetter = new PositionVoltage(0);              // 轉向位置閉環命令物件
    m_velocitySetter.Slot = 0;                           // 使用 Slot0 參數（對應 ConsSwerve.driveGains）
    m_angleSetter.Slot = 0;                              // 使用 Slot0 參數（對應 ConsSwerve.rotorGains）

    TalonFXConfiguration talonConfigs = new TalonFXConfiguration(); // 建立通用設定容器

    // === 行走馬達設定 ===
    m_throttle.getConfigurator().apply(talonConfigs);     // 先套預設（清理）
    m_throttle.getConfigurator().apply(ConsSwerve.driveGains); // 再套行走 PID 參數

    // === 轉向馬達設定 ===
    talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();      // 可選：扭矩電流設定（保留）
    talonConfigs.Feedback.FeedbackRemoteSensorID = module.encoder_id;          // 轉向閉環回授：用遠端 CANcoder
    talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; // 指定來源為 Fused CANcoder
    talonConfigs.Feedback.RotorToSensorRatio = 150.0 / 7.0;       // ✅ 用 double：齒比 150/7（避免整數除法成 21）
    talonConfigs.ClosedLoopGeneral.ContinuousWrap = true;         // 允許角度連續換算（-∞~+∞ 環狀）
    talonConfigs.Slot0 = ConsSwerve.rotorGains;                   // 套用轉向 PID 參數（Slot0）
    talonConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // 轉向馬達方向（依機構而定）
    m_rotor.getConfigurator().apply(talonConfigs);                // 套用到轉向馬達

    // === 煞車/滑行模式 ===
    m_throttle.setNeutralMode(NeutralModeValue.Brake);   // 行走：煞車（鬆開會煞停）
    m_rotor.setNeutralMode(NeutralModeValue.Brake);      // 轉向：滑行（避免抖動）

    // === CANcoder 設定 ===
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = module.offset;              // 絕對零點校正（你的常數表中已計算）
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;    // 0.5 代表 0~1 斷點在 0.5 轉（避免臨界跳變）
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_encoder.getConfigurator().apply(config);                     // 套用設定
  }

  // === 設定目標狀態（速度 + 角度） ===
  public void setState(SwerveModuleState state) {
    // ✅ 關鍵修正：一定要接回 optimize 的回傳值，讓模組走最短路徑（必要時把速度取負、角度 ±180°）
    state.optimize(getAngle());

    // 依你定義的換算係數，把 m/s 轉成馬達 closed-loop 的「機構轉速單位」
    double throttleOutput = state.speedMetersPerSecond / ConsSwerve.kThrottleConversionFactor;

    // 角度轉成「轉數」：WPILib 的 Rotation2d.getRotations() 回傳 [0,1) 轉
    double rotorOutput = state.angle.getRotations();

    // 行走：速度閉環
    m_throttle.setControl(m_velocitySetter.withVelocity(throttleOutput));

    // 轉向：位置閉環（搭配 ContinuousWrap，可直接跨 ±π）
    m_rotor.setControl(m_angleSetter.withPosition(rotorOutput));
  }

  // === 取得目前模組角度（相對角，供 optimize 與控制使用）===
  public Rotation2d getAngle() {
    // Phoenix 6 的 CANcoder Position：通常為「可重設的機構位置(轉數)」
    // 你這裡採用 Position 作為轉向目前角度，與 FusedCANcoder 回授一致，適合控制閉環使用
    return Rotation2d.fromRotations(m_encoder.getPosition().getValueAsDouble());
  }

  // === 回報目前模組狀態（行走速度 + 角度）===
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_throttle.getVelocity().getValueAsDouble() * ConsSwerve.kThrottleConversionFactor, // 轉回 m/s
        getAngle()                                                                          // 現在朝向
    );
  }

  // === 回報里程計所需的「輪組里程 + 角度」===
  public SwerveModulePosition getPosition() {
    // 馬達 Position 轉回「輪子前進的米數」
    double throttlePosition = m_throttle.getPosition().getValueAsDouble() * ConsSwerve.kThrottleConversionFactor;

    // 里程計用「絕對角度」較穩（不受你在程式中重設 Position 影響）
    return new SwerveModulePosition(
        throttlePosition,
        Rotation2d.fromRotations(m_encoder.getAbsolutePosition().getValueAsDouble())
    );
  }
}
