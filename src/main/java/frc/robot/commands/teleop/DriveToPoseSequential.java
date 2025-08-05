package frc.robot.commands.teleop;
// package frc.robot.commands;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathPoint;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.subsystems.DriveTrain;
// import com.pathplanner.lib.path.PathPoint;

// import java.util.List;

// public class DriveToPoseSequential {
//     public static Command create(DriveTrain driveTrain, Pose2d targetPose, PathConstraints constraints) {
//         Pose2d currentPose = driveTrain.getPose();

//         // 先轉向
//         Command turnToAngle = new StartEndCommand(
//             () -> {
//                 double current = driveTrain.getPose().getRotation().getDegrees();
//                 double target = targetPose.getRotation().getDegrees();
//                 double error = target - current;
//                 error = Math.IEEEremainder(error, 360.0); // wrap 處理
//                 double zOutput = driveTrain.vz.calculate(0, -error);
//                 zOutput = Math.copySign(Math.min(Math.abs(zOutput), 1.0), zOutput);
//                 driveTrain.drive(0, 0, zOutput);
//             },
//             driveTrain::stopModules,
//             driveTrain
//         ).until(() -> {
//             double angleError = targetPose.getRotation()
//                 .minus(driveTrain.getPose().getRotation())
//                 .getDegrees();
//             return Math.abs(angleError) < 1.5;
//         }).withTimeout(1.5);

//         // 建立 PathPoint（新版寫法）
//         PathPoint start = PathPoint.fromCurrentHolonomicState(
//             currentPose.getTranslation(),
//             currentPose.getRotation(),   // heading
//             currentPose.getRotation()    // 車頭朝向
//         );

//         PathPoint end = PathPoint.fromHolonomicPose(
//             targetPose.getTranslation(),
//             targetPose.getRotation(),   // heading
//             targetPose.getRotation()    // 車頭朝向
//         );

//         // 建立動態 Path
//         PathPlannerPath path = PathPlannerPath.fromPathPoints(
//             List.of(start, end),
//             constraints,
//             new GoalEndState(0.0, targetPose.getRotation())
//         );

//         // 跟隨路徑
//         Command followPath = AutoBuilder.followPath(path);

//         return new SequentialCommandGroup(
//             turnToAngle,
//             followPath
//         );
//     }
// }