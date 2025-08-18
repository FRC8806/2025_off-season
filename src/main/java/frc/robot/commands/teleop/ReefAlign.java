package frc.robot.commands.teleop;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import com.studica.frc.AHRS;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Tools;
import frc.robot.constants.ConsAuto;
import frc.robot.constants.ConsController;
import frc.robot.constants.ConsSwerve;
import frc.robot.subsystems.DriveTrain;

public class ReefAlign extends Command {
  private final DriveTrain m_driveTrain;
  // private final Vision m_vision;
  public AHRS m_gyro;

  private final Supplier<Boolean> r1, r2, r3, r10, r11, r12;
  private final Supplier<Double> r4, r5, r6, r7, r8, r9;
  private final Supplier<Double> xAxis, yAxis, zAxis;
  private final Supplier<Boolean> isRed;

  private Command pathCommand = null;
  private ConsAuto.PositionName currentTarget = null;

  public ReefAlign(
      DriveTrain m_driveTrain,
      // Vision m_vision,
      Supplier<Boolean> r1,
      Supplier<Boolean> r2,
      Supplier<Boolean> r3,
      Supplier<Double> r4,
      Supplier<Double> r5,
      Supplier<Double> r6,
      Supplier<Double> r7,
      Supplier<Double> r8,
      Supplier<Double> r9,
      Supplier<Boolean> r10,
      Supplier<Boolean> r11,
      Supplier<Boolean> r12,
      Supplier<Double> xAxis,
      Supplier<Double> yAxis,
      Supplier<Double> zAxis,
      Supplier<Boolean> isRed) {
    this.m_driveTrain = m_driveTrain;

    this.r1 = r1;
    this.r2 = r2;
    this.r3 = r3;
    this.r4 = r4;
    this.r5 = r5;
    this.r6 = r6;
    this.r7 = r7;
    this.r8 = r8;
    this.r9 = r9;
    this.r10 = r10;
    this.r11 = r11;
    this.r12 = r12;

    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.zAxis = zAxis;
    this.isRed = isRed;

  }

  @Override
  public void initialize() {
    ConsAuto.setAllianceColor(isRed.get());
    SmartDashboard.putString("AT2_state", "initialized");
  }

  @Override
  public void execute() {
    ConsAuto.PositionName target = getPressedTarget();
    boolean joystickMoving = Math.abs(xAxis.get()) > 0.05 || Math.abs(yAxis.get()) > 0.05
        || Math.abs(zAxis.get()) > 0.05;

    if (target != null) {
      if (!target.equals(currentTarget) || pathCommand == null || pathCommand.isFinished()) {
        startPathToTarget(target);
      }
      SmartDashboard.putString("AT2_state", "following");
      return;
    }

    if (pathCommand != null) {
      if (pathCommand.isScheduled())
        pathCommand.cancel();
      pathCommand = null;
    }
    currentTarget = null;

    if (joystickMoving) {
      m_driveTrain.drive(
          -getDriveControllerAxisOnDeadBand(yAxis.get(), 2) * ConsSwerve.throttleMaxSpeed,
          -getDriveControllerAxisOnDeadBand(xAxis.get(), 2) * ConsSwerve.throttleMaxSpeed,
          getDriveControllerAxisOnDeadBand(zAxis.get(), 2) * ConsSwerve.kMaxRotationSpeed);
      SmartDashboard.putString("AT2_state", "manual-driving");
    } else {
      m_driveTrain.stopModules();
      SmartDashboard.putString("AT2_state", "hold");
    }
  }


  private void startPathToTarget(ConsAuto.PositionName target) {
    SmartDashboard.putString("AT2_startTarget", target.name());

    if (pathCommand != null && pathCommand.isScheduled()) {
      pathCommand.cancel();
    }

    currentTarget = target;
    Pose2d targetPose = ConsAuto.pos(target);
    SmartDashboard.putNumber("AT2_goalX", targetPose.getX());
    SmartDashboard.putNumber("AT2_goalY", targetPose.getY());
    SmartDashboard.putNumber("AT2_goalZdeg", targetPose.getRotation().getRotations());

    Pose2d start = m_driveTrain.getPose();
    Pose2d end = targetPose;

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, end);

    PathConstraints constraints = new PathConstraints(
        5.6, // max linear vel (m/s)
        5, // max linear accel (m/s^2)
        Math.PI, // max angular vel (rad/s) —— 無所謂，Z 你自己控
        2 * Math.PI // max angular accel (rad/s^2)
    );

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        new GoalEndState(0, end.getRotation()));
    path.preventFlipping = true;

    SmartDashboard.putBoolean("AT2_setHeadingSupplier", true);

    pathCommand = AutoBuilder.followPath(path);
    pathCommand.schedule();
    SmartDashboard.putBoolean("AT2_pathScheduled", true);
    SmartDashboard.putString("AT2_state", "following");
  }

  private void cancelPathAndReturnToManual() {
    if (pathCommand != null && pathCommand.isScheduled()) {
      pathCommand.cancel();
    }
    pathCommand = null;
    currentTarget = null;

    SmartDashboard.putString("AT2_state", "manual");

    m_driveTrain.drive(
        -getDriveControllerAxisOnDeadBand(yAxis.get(), 2) * ConsSwerve.throttleMaxSpeed,
        -getDriveControllerAxisOnDeadBand(xAxis.get(), 2) * ConsSwerve.throttleMaxSpeed,
        getDriveControllerAxisOnDeadBand(zAxis.get(), 2) * ConsSwerve.kMaxRotationSpeed);
  }

  private ConsAuto.PositionName getPressedTarget() {
    if (r1.get())
      return ConsAuto.PositionName.r1;
    if (r2.get())
      return ConsAuto.PositionName.r2;
    if (r3.get())
    return ConsAuto.PositionName.r3;
    if (r4.get() == 1)
    return ConsAuto.PositionName.r4;
    if (r5.get() == 1)
    return ConsAuto.PositionName.r5;
    if (r6.get() == -1)
    return ConsAuto.PositionName.r6;
    if (r7.get() == 1)
    return ConsAuto.PositionName.r7;
    if (r8.get() == 1)
    return ConsAuto.PositionName.r8;
    if (r9.get() == -1)
    return ConsAuto.PositionName.r9;
    if (r10.get())
    return ConsAuto.PositionName.r10;
    if (r11.get())
    return ConsAuto.PositionName.r11;
    if (r12.get())
    return ConsAuto.PositionName.r12;
    return null;
  }

  @Override
  public void end(boolean interrupted) {
    if (pathCommand != null && pathCommand.isScheduled()) {
      pathCommand.cancel();
    }
    pathCommand = null;

    m_driveTrain.stopModules(); // 等同 drive(new ChassisSpeeds())

    SmartDashboard.putString("AT2_state", interrupted ? "ended(interrupted)" : "ended");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double getDriveControllerAxisOnDeadBand(double value, int power) {
    boolean isValueNegtive = value < 0;
    value = Math.abs(value) > ConsController.DEADBAND ? Math.abs(value) - ConsController.DEADBAND : 0;
    value = Math.pow(value, power);
    value = Tools.map(value, 0, Math.pow(1 - ConsController.DEADBAND, power), 0, 1);
    return isValueNegtive ? -value : value;
  }
}
