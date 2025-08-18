package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

public class ConsLift {
  // ID
  public static final int LIFT_MOTOR_ID = 21;//
  public static final int ARM_ANGLE_MOTOR_ID = 22;//
  public static final int ROLLING_MOTOR_ID = 23;//
  public static final int ARM_CANCODER_ID = 5;

  // speed
  public static final double coralSpeed = 0.95;//
  public static final double algaeSpeed = 0.8;//

  // PID
  private static final Slot0Configs liftSlot0 = new Slot0Configs().withKP(1.6).withKI(0.1).withKD(0.1).withKG(0.8);//

  // limit
  private static final SoftwareLimitSwitchConfigs armAngleLimit = new SoftwareLimitSwitchConfigs()
      .withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(0)// true
      .withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(-0.4);// true

  private static final SoftwareLimitSwitchConfigs liftLimit = new SoftwareLimitSwitchConfigs()
      .withForwardSoftLimitEnable(false).withForwardSoftLimitThreshold(0)// true
      .withReverseSoftLimitEnable(false).withReverseSoftLimitThreshold(0);// true

  // voltage limit
  private static final VoltageConfigs armAngleVoltageLimit = new VoltageConfigs().withPeakForwardVoltage(6)
      .withPeakReverseVoltage(-6);// need to test

  // config
  public static final TalonFXConfiguration armAngleConfigs = new TalonFXConfiguration()
      .withSoftwareLimitSwitch(armAngleLimit).withVoltage(armAngleVoltageLimit);
  public static final TalonFXConfiguration liftConfigs = new TalonFXConfiguration().withSlot0(liftSlot0)
      .withSoftwareLimitSwitch(liftLimit);

  public enum Pose {
    L1(-18, -0.145, 10, false), //
    L2(-5.879, -0.2687, 0.2, true), // 大//-5.879, -12.0 / 12.15, 5/12.15,
    L3(-29.8, -0.304991, 10, false), // 大
    L4(-58, -0.360921, 40, false), // 大
    RESET_C(-12, 0, 0.2, true), // 中
    L2A(-20.678, -0.215, 5, false), // 中
    L3A(-50, -0.215, 10, false), // 中
    Put_A(-58.9, -0.425, 0.2, true), // 中
    Put_a(-12, -0.134, 0.08, true), // 中
    DOWM_CORAL(-0.2, 0, 0.02, true), // 小小//-0.5, -0.6 / 12.15, 1/12.15,
    climber(-5, -0.08, -0.23, true), // -5, -6.0 / 12.15, 1/12.15
    UP_CORAL(-12, 0, -0.6, false);// 小

    public final double pos_lift, pos_arm, range;
    public final boolean armFirst;

    Pose(double pos_lift, double pos_arm, double range, boolean armFirst) {
      this.pos_lift = pos_lift;
      this.pos_arm = pos_arm;
      this.range = range;
      this.armFirst = armFirst;
    }
  }
}
