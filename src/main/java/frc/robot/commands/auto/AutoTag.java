package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsAuto;
import frc.robot.subsystems.DriveTrain;

import java.util.List;

import edu.wpi.first.math.geometry.*;

public class AutoTag extends Command {
    private final DriveTrain driveTrain;
    private final ConsAuto.Position position;
    private Command pathCommand;

    public AutoTag(DriveTrain driveTrain, ConsAuto.Position position) {
        this.driveTrain = driveTrain;
        this.position = position;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        Pose2d start = driveTrain.getPose();
        Pose2d end = new Pose2d(position.x, position.y, Rotation2d.fromDegrees(position.z));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            start,
            new Pose2d(position.x, position.y, start.getRotation()), // 中繼旋轉
            end
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, Math.PI, 2 * Math.PI);
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, end.getRotation())
        );

        path.preventFlipping = true;
        pathCommand = AutoBuilder.followPath(path);
        pathCommand.schedule();
    }

    @Override
    public void execute() {
        // 路徑已由 pathCommand 處理，這裡不需要額外行為
    }

    @Override
    public boolean isFinished() {
        return pathCommand == null || pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) pathCommand.cancel();
    }
}
