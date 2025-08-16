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
// import frc.robot.subsystems.Vision;
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
    // this.m_vision = m_vision;

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

    // addRequirements(m_vision);
  }

  @Override
  public void initialize() {
    ConsAuto.setAllianceColor(isRed.get());
    // 清掉任何舊的期望角度
    // m_driveTrain.clearDesiredHeadingSupplier();
    SmartDashboard.putString("AT2_state", "initialized");
  }

  @Override
  public void execute() {
    ConsAuto.PositionName target = getPressedTarget();
    boolean joystickMoving = Math.abs(xAxis.get()) > 0.05 || Math.abs(yAxis.get()) > 0.05
        || Math.abs(zAxis.get()) > 0.05;

    if (target != null) {
      // 有按鈕：只跑自動，忽略搖桿
      if (!target.equals(currentTarget) || pathCommand == null || pathCommand.isFinished()) {
        startPathToTarget(target);
      }
      SmartDashboard.putString("AT2_state", "following");
      return;
    }

    // ===== 沒按鈕：把自動真的取消掉 =====
    if (pathCommand != null) {
      if (pathCommand.isScheduled())
        pathCommand.cancel();
      pathCommand = null;
    }
    currentTarget = null;
    // m_driveTrain.clearDesiredHeadingSupplier();

    if (joystickMoving) {
      // 純手動
      m_driveTrain.drive(
          -getDriveControllerAxisOnDeadBand(yAxis.get(), 2) * ConsSwerve.throttleMaxSpeed,
          -getDriveControllerAxisOnDeadBand(xAxis.get(), 2) * ConsSwerve.throttleMaxSpeed,
          getDriveControllerAxisOnDeadBand(zAxis.get(), 2) * ConsSwerve.kMaxRotationSpeed);
      SmartDashboard.putString("AT2_state", "manual-driving");
    } else {
      // 沒按鈕 + 沒搖桿：完全不動
      m_driveTrain.stopModules();
      SmartDashboard.putString("AT2_state", "hold");
    }
  }

  // private void stopAndHold() {
  //   if (pathCommand != null && pathCommand.isScheduled()) {
  //     pathCommand.cancel();
  //   }
  //   pathCommand = null;
  //   currentTarget = null;

  //   // 不再給期望角度，避免 autoDrive 再輸出旋轉
  //   // m_driveTrain.clearDesiredHeadingSupplier();

  //   // 速度清 0，機器人不動
  //   m_driveTrain.stopModules(); // 等同 drive(new ChassisSpeeds());

  //   SmartDashboard.putString("AT2_state", "hold");
  // }

  private void startPathToTarget(ConsAuto.PositionName target) {
    SmartDashboard.putString("AT2_startTarget", target.name());

    // 取消舊 path
    if (pathCommand != null && pathCommand.isScheduled()) {
      pathCommand.cancel();
    }

    currentTarget = target;
    Pose2d targetPose = ConsAuto.pos(target);
    SmartDashboard.putNumber("AT2_goalX", targetPose.getX());
    SmartDashboard.putNumber("AT2_goalY", targetPose.getY());
    SmartDashboard.putNumber("AT2_goalZdeg", targetPose.getRotation().getRotations());

    // 只跑 XY；終點 rotation = 現在角度，避免 PP 幫你轉
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

    // 把期望車頭角度(deg)交給底盤，由 DriveTrain.autoDrive() 的 PID 輸出 ω
    // m_driveTrain.setDesiredHeadingSupplier(() -> ConsAuto.pos(currentTarget).z);
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

    // 回到手動時，不要再鎖角度
    // m_driveTrain.clearDesiredHeadingSupplier();
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
    // 1) 確保路徑真的停掉
    if (pathCommand != null && pathCommand.isScheduled()) {
      pathCommand.cancel();
    }
    pathCommand = null;

    // 2) 先把「目標朝向」關掉，避免 autoDrive 用 vz 再輸出 ω
    // m_driveTrain.clearDesiredHeadingSupplier();

    // 如有用過 overrideRotationFeedback，順手清掉（安全起見）
    // PPHolonomicDriveController.overrideRotationFeedback(null);

    // 3) 停車（清 0 速度）
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
