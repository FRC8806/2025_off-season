package frc.robot.commands.teleop;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import com.studica.frc.AHRS;

import java.util.List;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Tools;
import frc.robot.constants.ConsAuto;
import frc.robot.constants.ConsController;
import frc.robot.constants.ConsSwerve;
import frc.robot.constants.ConsLift.Pose;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.VirtualPose;
 import frc.robot.subsystems.Vision;

import java.util.List;
import java.util.function.Supplier;

public class AprilTag2 extends Command {
  private final DriveTrain m_driveTrain;
   private final Vision m_vision;
    public AHRS m_gyro;

  //private final VirtualPose m_VirtualPose;

  private final Supplier<Boolean> r1, r2, r3, r10, r11, r12;
  private final Supplier<Double> r4, r5, r6, r7, r8, r9;
  private final Supplier<Double> xAxis, yAxis, zAxis;
  private final Supplier<Boolean> isRed;

  private Command pathCommand = null;
  private String currentTarget = null;

  public 
  AprilTag2(
    DriveTrain m_driveTrain,
    //VirtualPose m_VirtualPose,
    Vision m_vision,


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
    Supplier<Boolean> isRed
  ) {
    this.m_driveTrain = m_driveTrain;
    //this.m_VirtualPose = m_VirtualPose;
    this.m_vision = m_vision;
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
    addRequirements(m_driveTrain, m_vision);
  }

  @Override
  public void initialize() {
    ConsAuto.setAllianceColor(isRed.get());
  }

//   @Override
//   public void execute() {
//     String target = getPressedTarget();

//     if (target != null) {
//       // 如果目標改變，或沒有正在跑 path，則建立新 path
//       if (!target.equals(currentTarget) || pathCommand == null || pathCommand.isFinished()) {
       
//         // pathCommand = AutoBuilder.followPath(path);
//         currentTarget = target;
//         ConsAuto.Position pos = ConsAuto.pos(target);

       


// // 建立中繼點（旋轉方向建議沿著目標方向）
// Pose2d start = m_driveTrain.getPose();
// Pose2d end = new Pose2d(pos.x, pos.y,  start.getRotation());

// List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start
// , end);


// PathConstraints constraints = new PathConstraints(
//          3,           // Max linear velocity (m/s)
//          3,          // Max linear acceleration (m/s²)
//            Math.PI,       // Max angular velocity
//             2 * Math.PI    // Max angular acceleration
//           );

// PathPlannerPath path = new PathPlannerPath(
//     waypoints,
//     constraints, // 速度/加速度限制
//     null,
    
//     new GoalEndState(0.0, start.getRotation())
//     //new GoalEndState(0.5,m_driveTrain.getDiretion())
// );

// // 執行

//         //List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, end);

//         path.preventFlipping = true;
//        Command pathCommand = AutoBuilder.followPath(path);
//         pathCommand.schedule(); 
//       }
     
//       // 執行 path
//       // if (pathCommand != null) {
//       //   pathCommand.execute();
//       // }

//     } else {
//       // 沒有按 r1~r12，切回手動控制
//       currentTarget = null;
//       pathCommand = null;

//       m_driveTrain.drive(
//         -getDriveControllerAxisOnDeadBand(yAxis.get(), 2) * ConsSwerve.throttleMaxSpeed,
//         -getDriveControllerAxisOnDeadBand(xAxis.get(), 2) * ConsSwerve.throttleMaxSpeed,
//          getDriveControllerAxisOnDeadBand(zAxis.get(), 2) * ConsSwerve.kMaxRotationSpeed
//       );
//     }
//   }

@Override
public void execute() {
    String target = getPressedTarget(); // null or "r1"~"r12"
    boolean joystickMoving = Math.abs(xAxis.get()) > 0.05 || Math.abs(yAxis.get()) > 0.05 || Math.abs(zAxis.get()) > 0.05;

    if (joystickMoving) {
        // Joystick moved: cancel auto-alignment if any
        if (pathCommand != null && pathCommand.isScheduled()) {
            pathCommand.cancel();
        }
        pathCommand = null;
        currentTarget = null;

        // Manual drive
        m_driveTrain.drive(
            -getDriveControllerAxisOnDeadBand(yAxis.get(), 2) * ConsSwerve.throttleMaxSpeed,
            -getDriveControllerAxisOnDeadBand(xAxis.get(), 2) * ConsSwerve.throttleMaxSpeed,
            getDriveControllerAxisOnDeadBand(zAxis.get(), 2) * ConsSwerve.kMaxRotationSpeed
        );
        return;
    }

    // If joystick not moving
    if (target != null) {
        if (!target.equals(currentTarget) || pathCommand == null || pathCommand.isFinished()) {
            // Start new path
            currentTarget = target;
            ConsAuto.Position pos = ConsAuto.pos(target);
            Pose2d start = m_driveTrain.getPose();
            Pose2d end = new Pose2d(pos.x , pos.y, Rotation2d.fromDegrees(pos.z));//Rotation2d.fromDegrees(pos.z));
            
                        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start,
                        new Pose2d(pos.x, pos.y, start.getRotation()),
                         end);

                        PathConstraints constraints = new PathConstraints(3, 3, Math.PI, 2 * Math.PI);

                        PathPlannerPath path = new PathPlannerPath(
                            waypoints,
                            constraints,
                            null,
                            new GoalEndState(0.0, end.getRotation())
                        );
                        path.preventFlipping = true;
            
                        pathCommand = AutoBuilder.followPath(path);
                        pathCommand.schedule();
            currentTarget = target;
// ConsAuto.Position pos = ConsAuto.pos(target);

// // 取得起點與目標 Pose
// Pose2d start = m_driveTrain.getPose();
// Rotation2d targetRotation = Rotation2d.fromDegrees(pos.z);
// Pose2d end = new Pose2d(pos.x, pos.y, targetRotation);

// // 使用 fromHolonomicPose 指定移動方向 + 車頭方向（避免亂轉）
//                       List<Waypoint> waypoints = List.of(
//                           PathPoint.fromHolonomicPose(start, start.getRotation()),
//                           PathPoint.fromHolonomicPose(end, targetRotation)
//                       );
//                       // 設定速度與旋轉限制
//                       PathConstraints constraints = new PathConstraints(3, 3, Math.PI, 2 * Math.PI);

//                       // 建立 path 並指定目標車頭方向
//                       PathPlannerPath path = new PathPlannerPath(
//                           waypoints,
//                           constraints,
//                           null,
//                           new GoalEndState(0.0, targetRotation)
//                       );
                      // path.preventFlipping = true;

// // 開始執行路徑
// pathCommand = AutoBuilder.followPath(path);
// pathCommand.schedule();
                    }
                }
                // else: not pressing any button but already in auto — continue following
            }
            
            
             
            
              private String getPressedTarget() {
    if (r1.get()) return "r1";
    if (r2.get()) return "r2";
    if (r3.get()) return "r3";
    if (r4.get() == 1) return "r4";//trigger
    if (r5.get() == 1) return "r5";
    if (r6.get() == -1) return "r6";
    if (r7.get() == 1) return "r7";
    if (r8.get() == 1) return "r8";
    if (r9.get() == -1) return "r9";
    if (r10.get()) return "r10";
    if (r11.get()) return "r11";
    if (r12.get()) return "r12";
    return null;
  }

  @Override
  public void end(boolean interrupted) {
    if (pathCommand != null) {
      pathCommand.end(interrupted);
    }
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

