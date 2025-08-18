package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConsLift;

public class Lift extends SubsystemBase {
  private final TalonFX liftMotor = new TalonFX(ConsLift.LIFT_MOTOR_ID);
  private final TalonFX armAngleMotor = new TalonFX(ConsLift.ARM_ANGLE_MOTOR_ID);
  private final TalonFX rollingMotor = new TalonFX(ConsLift.ROLLING_MOTOR_ID);
  private final CANcoder armSensor = new CANcoder(ConsLift.ARM_CANCODER_ID);

  private PositionVoltage positionSetter = new PositionVoltage(0);

  private double armSensorOffset = -0.06;
  // private final PIDController armPID = new PIDController(2.5, 0.45, 0.1);// 3.6 0.3 0.2
  private double armTargetPosition = 0;
  private boolean useArmPID = false;

  private boolean get = false, armGet = false;

  public Lift() {
    // Motor Config
    liftMotor.getConfigurator().apply(ConsLift.liftConfigs);
    liftMotor.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration armAngleConfigs = ConsLift.armAngleConfigs;
    armAngleConfigs.Feedback.FeedbackRemoteSensorID = ConsLift.ARM_CANCODER_ID;
    armAngleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    armAngleConfigs.Feedback.RotorToSensorRatio = 54.64;
    armAngleConfigs.Slot0 = new Slot0Configs().withKP(25).withKI(6).withKD(0.01);
    armAngleMotor.getConfigurator().apply(ConsLift.armAngleConfigs);
    armAngleMotor.setNeutralMode(NeutralModeValue.Brake);

    rollingMotor.setNeutralMode(NeutralModeValue.Coast);

    // CANcoder Offset
    CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
    encoderConfigs.MagnetSensor.MagnetOffset = armSensorOffset;
    armSensor.getConfigurator().apply(encoderConfigs);

    positionSetter.Slot = 0;

    // PID tolerance
    // armPID.setTolerance(0.2);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift Position", getLiftPosition());
    SmartDashboard.putNumber("Arm Position (CANcoder)", getArmPosition());
    SmartDashboard.putNumber("Arm Raw Sensor", getArmPositionFromSensor());
    SmartDashboard.putNumber("Rolling Speed", getRollingSpeed());
    SmartDashboard.putBoolean("Arm PID running", useArmPID);
    SmartDashboard.putNumber("Arm PID Target", armTargetPosition);
    SmartDashboard.putNumber("Arm PID Error", Math.abs(getArmPosition() - armTargetPosition));



    // updateArmPIDControl(RobotState.isEnabled());
  }

  // ========== CANcoder ==========

  public double getArmPositionFromSensor() {
    return armSensor.getPosition().getValueAsDouble();
  }

  public double getArmPosition() {
    return getArmPositionFromSensor();
  }
  // ========== Arm PID 控制 ==========

  public void setArmPosition(double position) {
    // armTargetPosition = position;
    // useArmPID = true;
    armAngleMotor.setControl(positionSetter.withPosition(position));
  }

  // private void updateArmPIDControl(Boolean isEnabled) {
  //   if (isEnabled) {
  //     if (useArmPID) {
  //       double current = getArmPosition();
  //       double output = armPID.calculate(current, armTargetPosition);

  //       // 持續輸出 PID 結果，不要停止
  //       armAngleMotor.set(output);

  //       // 可選：如果需要知道是否已到達目標，可用這個方法
  //       // 但不要停止 PID 控制
  //       if (armPID.atSetpoint()) {
  //         // 已到達目標，但仍繼續 PID 控制以維持位置
  //         SmartDashboard.putBoolean("Arm At Target", true);
  //       } else {
  //         SmartDashboard.putBoolean("Arm At Target", false);
  //       }
  //     }
  //   }
  // }

  // 新增：停止 PID 控制的方法（只在真的需要停止時使用）
  // public void stopArmPID() {
  //   useArmPID = false;
  //   armAngleMotor.set(0);
  // }

  // 新增：檢查是否到達目標位置
  // public boolean isArmAtTarget() {
  //   return armPID.atSetpoint();
  // }

  // ========== 手動控制 ==========

  public void setLiftSpeed(double speed) {
    liftMotor.set(speed);
  }

  // public void setArmSpeed(double speed) {
  //   armAngleMotor.set(speed);
  // }

  public void setRollingSpeed(double speed) {
    rollingMotor.set(speed);
  }

  public double getLiftPosition() {
    return liftMotor.getPosition().getValueAsDouble();
  }

  public double getRollingSpeed() {
    return rollingMotor.getVelocity().getValueAsDouble();
  }

  public void setLiftPosition(double position) {
    liftMotor.setControl(
        new com.ctre.phoenix6.controls.PositionVoltage(position).withSlot(0));
  }

  // public void setArmPosition(double position) {
  //   setArmTarget(position);
  // }

  // ========== 定位控制（Pose） ==========

  public void setPose(ConsLift.Pose pose) {
    double lift = pose.pos_lift, arm = pose.pos_arm, range = pose.range;
    if (pose.armFirst) {
      setArmPosition(arm);
      if (Math.abs(arm - getArmPosition()) <= range)
        setLiftPosition(lift);
    } else {
      setLiftPosition(lift);
      if (Math.abs(lift - getLiftPosition()) <= range)
        setArmPosition(arm);
    }
  }

  public void set(ConsLift.Pose pose) {
    double lift = pose.pos_lift, arm = pose.pos_arm, range = pose.range;
    if (pose.armFirst) {
      setArmPosition(arm);
      setRollingSpeed(0.8);
      if (Math.abs(arm - getArmPosition()) <= range)
        setLiftPosition(lift);
    } else {
      setLiftPosition(lift);
      if (Math.abs(lift - getLiftPosition()) <= range)
        setArmPosition(arm);
    }
  }

  public boolean isFinished(ConsLift.Pose pose) {
    return Math.abs(pose.pos_arm - getArmPosition()) < 0.5 &&
        Math.abs(pose.pos_lift - getLiftPosition()) < 2;
  }

  // ========== 自動 Coral 操作邏輯 ==========

  public boolean getCoral() {
    if (!get) {
      setPose(ConsLift.Pose.DOWM_CORAL);
      setRollingSpeed(ConsLift.coralSpeed);
      if (isFinished(ConsLift.Pose.DOWM_CORAL)) {
        get = true;
        armGet = false;
      }
    } else {
      setRollingSpeed(0);
      if (isFinished(ConsLift.Pose.DOWM_CORAL)) {
        get = false;
        armGet = true;
      }
    }
    return armGet;
  }

}
