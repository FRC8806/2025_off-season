package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.constants.ConsAuto;
import frc.robot.subsystems.DriveTrain;

import java.util.List;
import java.util.Set;

public class AutoTagToTag extends SequentialCommandGroup {

  private static final PathConstraints PATH = new PathConstraints(
      5.6, // max linear vel (m/s)
      5.0, // max linear accel (m/s^2)
      Math.PI, // max angular vel (rad/s)
      2 * Math.PI // max angular accel (rad/s^2)
  );

  private final DriveTrain driveTrain;
  private final ConsAuto.PositionName posName;

  public AutoTagToTag(DriveTrain driveTrain, ConsAuto.PositionName posName) {
    this.driveTrain = driveTrain;
    this.posName = posName;
    build();
  }

  private void build() {
    addRequirements(driveTrain);

    addCommands(
        Commands.runOnce(() -> {
          var alliance = DriverStation.getAlliance();
          boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          ConsAuto.setAllianceColor(isRed);
          SmartDashboard.putBoolean("AutoTag/isRed", isRed);
        }, driveTrain),

        Commands.defer(() -> {
          Pose2d start = driveTrain.getPose();
          Pose2d target = ConsAuto.getPosition(posName);

          Rotation2d endRot = target.getRotation();
          Pose2d end = new Pose2d(target.getX(), target.getY(), endRot);

          List<Waypoint> wps = PathPlannerPath.waypointsFromPoses(start, end);

          PathPlannerPath path = new PathPlannerPath(
              wps,
              PATH,
              null,
              new GoalEndState(0.8, endRot));
          path.preventFlipping = true;

          SmartDashboard.putNumber("AutoTag/goalX", target.getX());
          SmartDashboard.putNumber("AutoTag/goalY", target.getY());
          SmartDashboard.putNumber("AutoTag/goalDeg", target.getRotation().getDegrees());

          return AutoBuilder.followPath(path);
        }, Set.of(driveTrain)),

        Commands.runOnce(() -> {
          driveTrain.stopModules();
          SmartDashboard.putString("AutoTag/state", "done");
        }, driveTrain));
  }
}
