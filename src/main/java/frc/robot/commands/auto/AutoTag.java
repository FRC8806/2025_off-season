package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.constants.ConsAuto;
import frc.robot.constants.ConsSwerve;
import frc.robot.subsystems.DriveTrain;

import java.util.List;
import java.util.Set;

public class AutoTag extends SequentialCommandGroup {

  // 路徑約束
  private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
      ConsSwerve.throttleMaxSpeed,
      5.0,
      Math.PI,
      2.0 * Math.PI
  );

  // 精度要求（只看 XY）
  private static final double FINISH_TOL_M  = 0.01; // 0.8 cm
  private static final double STABLE_TIME_S = 0.15;  // 

  // creep 收尾參數
  private static final double XY_KP          = 2.0;
  private static final double CREEP_V_MIN    = 0.10;
  private static final double CREEP_V_MAX    = Math.max(0,0);
  private static final double DECEL_RADIUS_M = 0.60;

  private final DriveTrain driveTrain;
  private final ConsAuto.PositionName posName; // 依聯盟鏡像
  // private final ConsAuto.Pose2d fixedPos;    // 不鏡像

  public AutoTag(DriveTrain driveTrain, ConsAuto.PositionName posName) {
    this.driveTrain = driveTrain;
    this.posName = posName;
    // this.fixedPos = null;
    build();
  }

  // public AutoTag(DriveTrain driveTrain, ConsAuto.Pose2d position) {
  //   this.driveTrain = driveTrain;
  //   this.posName = null;
  //   this.fixedPos = position;
  //   build();
  // }

  private void build() {
    addRequirements(driveTrain);

    addCommands(
      // A) 同步聯盟色（避免雙重鏡像）
      Commands.runOnce(() -> {
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        ConsAuto.setAllianceColor(isRed);
        SmartDashboard.putBoolean("AutoTag/isRed", isRed);
      }, driveTrain),

      // B) 先用 PP 拉到附近：末速留一點避免睡死；終點角=起點角（但我們不控制 Z）
      Commands.defer(() -> {
        Pose2d start = driveTrain.getPose();
        Pose2d p = resolvePos();

        Rotation2d endRot = start.getRotation(); // PP 的終點角，僅避免切換時自轉
        Pose2d end = new Pose2d(p.getX(), p.getY(), endRot);

        List<Waypoint> wps = PathPlannerPath.waypointsFromPoses(start, end);

        PathPlannerPath path = new PathPlannerPath(
            wps,
            PATH_CONSTRAINTS,
            null,
            new GoalEndState(0.5, endRot) // 留 0.05 m/s 末速
        );
        path.preventFlipping = true;

        SmartDashboard.putNumber("AutoTag/targetX", p.getX());
        SmartDashboard.putNumber("AutoTag/targetY", p.getY());

        return AutoBuilder.followPath(path);
      }, Set.of(driveTrain)),

      // C) creep 收尾：只看 XY，角速度固定為 0
      Commands.run(() -> {
        Pose2d p = resolvePos();
        Pose2d cur = driveTrain.getPose();

        double dx = p.getX() - cur.getX();
        double dy = p.getY() - cur.getY();
        double dist = Math.hypot(dx, dy);

        double dirX = (dist > 1e-9) ? dx / dist : 0.0;
        double dirY = (dist > 1e-9) ? dy / dist : 0.0;

        double vCmd = XY_KP * dist;
        double slowDown = MathUtil.clamp(dist / DECEL_RADIUS_M, 0.10, 1.0);
        vCmd *= slowDown;
        vCmd = MathUtil.clamp(vCmd, CREEP_V_MIN, CREEP_V_MAX);

        double vx = vCmd * dirX; // 場地座標
        double vy = vCmd * dirY; // 場地座標
        double omega = 0.0;      // 不控制 Z

        driveTrain.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(vx, vy, omega));

        // 可視化
        SmartDashboard.putNumber("AutoTag/creep_dist_m", dist);
        SmartDashboard.putNumber("AutoTag/creep_vCmd", vCmd);
      }, driveTrain).until(new java.util.function.BooleanSupplier() {
        final double[] insideSince = { -1.0 };
        @Override public boolean getAsBoolean() {
          Pose2d p = resolvePos();
          Pose2d cur = driveTrain.getPose();

          double dist = Math.hypot(p.getX() - cur.getX(), p.getY() - cur.getY());
          double now = Timer.getFPGATimestamp();

          if (dist <= FINISH_TOL_M) {
            if (insideSince[0] < 0) insideSince[0] = now;
            return (now - insideSince[0]) >= STABLE_TIME_S;
          } else {
            insideSince[0] = -1.0;
            return false;
          }
        }
      }),

      // D) 收尾：停車
      Commands.runOnce(() -> {
        driveTrain.stopModules();
        SmartDashboard.putString("AutoTag/state", "done");
      }, driveTrain)
    );
  }

  private Pose2d resolvePos() {
    return ConsAuto.getPosition(posName);
  }
}
