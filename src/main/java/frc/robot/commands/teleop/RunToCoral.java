package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Yolo;

public class RunToCoral extends Command {
  private final DriveTrain drive;
  private final Yolo yolo;

  // ===== 參數 =====
  private static final double FAST_VX = 5.0;             // y<門檻時的追蹤速度 (m/s)
  private static final double MAX_VX = 5.0;              // 上限 (保險用)
  private static final double MAX_OMEGA = 2 * Math.PI; // 旋轉上限 (rad/s)
  private static final double ROT_SIGN = -1.0;           // x>0 時順時針需反向

  // 旋轉 PID（加快一點）
  private final PIDController rotPID = new PIDController(
      5,  // kP
      0.0,  // kI
      0.12  // kD
  );

  // ===== 切換與爆衝 =====
  private static final double Y_SWITCH = 0.0;            // 
  private static final double X_WINDOW = 0.05;           // |x| <= 0.10 才允許進入衝刺
  private static final double BURST_DISTANCE_M = 0.2;    // 衝刺固定距離 (m)
  private static final double BURST_VX = 1.5;            // 衝刺固定速度 (m/s)

  // 狀態
  private boolean bursting = false;
  private Pose2d burstStartPose = null;
  private Double burstHeadingRad = null;  // 鎖定衝刺起始角度
  private boolean done = false;

  public RunToCoral(DriveTrain m_drive, Yolo m_yolo) {
    this.drive = m_drive;
    this.yolo  = m_yolo;
    addRequirements(drive, yolo);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("rtc_on", true);
    rotPID.setSetpoint(0.0);
    rotPID.setTolerance(0.02);
    rotPID.reset();

    bursting = false;
    burstStartPose = null;
    burstHeadingRad = null;
    done = false;

    SmartDashboard.putString("RunToCoral/state", "init");
  }

  @Override
  public void execute() {
    if (!yolo.hasTarget()) {
      drive.drive(new ChassisSpeeds(0, 0, 0));
      SmartDashboard.putString("RunToCoral/state", "no_target");
      return;
    }

    final double x = yolo.getXc();
    final double y = yolo.getYc();
    if (!Double.isFinite(x) || !Double.isFinite(y)) {
      drive.drive(new ChassisSpeeds(0, 0, 0));
      SmartDashboard.putString("RunToCoral/state", "bad_input");
      return;
    }

    if (!bursting && y >= Y_SWITCH && Math.abs(x) <= X_WINDOW) {
      bursting = true;
      burstStartPose = drive.getPose();
      burstHeadingRad = drive.getDiretion().getRadians(); 
      SmartDashboard.putString("RunToCoral/state", "burst_start");
    }

    double omega;
    double vxRobot;

    if (bursting) {
      omega = 0.0;
      vxRobot = BURST_VX;

      Pose2d now = drive.getPose();
      double dx = now.getX() - burstStartPose.getX();
      double dy = now.getY() - burstStartPose.getY();
      double dist = Math.hypot(dx, dy);
      SmartDashboard.putNumber("RunToCoral/burst_dist", dist);

      double h = (burstHeadingRad != null) ? burstHeadingRad : drive.getDiretion().getRadians();
      double vxField = vxRobot * Math.cos(h);
      double vyField = vxRobot * Math.sin(h);
      drive.drive(new ChassisSpeeds(vxField, vyField, omega));

      if (dist >= BURST_DISTANCE_M) {
        done = true;
        SmartDashboard.putString("RunToCoral/state", "burst_done");
      } else {
        SmartDashboard.putString("RunToCoral/state", "bursting");
      }
      return;
    }

    // ===== 追蹤期：旋轉校正 + 高速前進 =====
    omega = MathUtil.clamp(ROT_SIGN * rotPID.calculate(x), -MAX_OMEGA, MAX_OMEGA);
    vxRobot = Math.min(FAST_VX, MAX_VX);

    double heading = drive.getDiretion().getRadians();
    double vxField = vxRobot * Math.cos(heading);
    double vyField = vxRobot * Math.sin(heading);
    drive.drive(new ChassisSpeeds(vxField, vyField, omega));

    SmartDashboard.putString("RunToCoral/state", "tracking");
    SmartDashboard.putNumber("RunToCoral/x", x);
    SmartDashboard.putNumber("RunToCoral/y", y);
    SmartDashboard.putNumber("RunToCoral/omega_cmd", omega);
    SmartDashboard.putNumber("RunToCoral/vx_cmd", vxRobot);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("rtc_on", false);
    drive.drive(new ChassisSpeeds(0, 0, 0));
    SmartDashboard.putString("RunToCoral/state", interrupted ? "interrupted" : "finished");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
