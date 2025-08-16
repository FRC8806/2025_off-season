package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;

public class ConsSwerve {
  public static final double halfChassisWidth = 0.3048;
  public static final double kWheelDiameter = 4 * 0.0254;

  public static final double gearRatio_L3plus = 5.35714286;// 5.36

  public static final double throttleMaxSpeed = 5800 / 60.0 / gearRatio_L3plus * kWheelDiameter * Math.PI;
  // public static final double kMaxRotationSpeed = throttleMaxSpeed / Math.sqrt(2
  // * Math.pow(0.5, 2)) * 2;
  // ConsSwerve.java
  public static final double kMaxRotationSpeed = 12; // rad/s，先測

  public static final Slot0Configs driveGains = new Slot0Configs().withKP(0.2).withKI(0.001).withKD(0.001);
  public static final Slot0Configs rotorGains = new Slot0Configs().withKP(20).withKI(0.0).withKD(0.001);
  // public static final Slot0Configs steerGains = new
  // Slot0Configs().withKV(0.12).withKP(0.4).withKI(0.001).withKD(0.001);

  public static final double kThrottleConversionFactor = kWheelDiameter * Math.PI / gearRatio_L3plus;

  public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(2.55, 0, 0.4);// 2 0 3
  public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(3.2, 0, 0.3);

  private static final double maxRadius = Math.hypot(halfChassisWidth, halfChassisWidth);

  // rad/s ✅
  public enum Modules {

    A(1, 2, 1, -1.5791015625, gearRatio_L3plus, new Translation2d(-halfChassisWidth, -halfChassisWidth)), // -1.5791015625
    B(3, 4, 2, -0.49267578125, gearRatio_L3plus, new Translation2d(-halfChassisWidth, halfChassisWidth)), // -0.49267578125
    C(5, 6, 3, 1.048828125, gearRatio_L3plus, new Translation2d(halfChassisWidth, -halfChassisWidth)), // 1.048828125
    D(7, 8, 4, 0.1396484375, gearRatio_L3plus, new Translation2d(halfChassisWidth, halfChassisWidth));// 0.1396484375

    public final int throttle_id, rotor_id, encoder_id;
    public final double offset, gearRatio;
    public final Translation2d pos;

    Modules(int t_id, int r_id, int e_id, double offset, double gearRatio, Translation2d pos) {
      this.throttle_id = t_id;
      this.rotor_id = r_id;
      this.encoder_id = e_id;
      this.offset = offset;
      this.gearRatio = gearRatio;
      this.pos = pos;
    }
  }
}
