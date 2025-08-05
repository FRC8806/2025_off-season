package frc.robot.commands.teleop;
// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Vision;
// import java.util.Comparator;
// import java.util.List;
// import java.util.Optional;
// import java.util.function.Supplier;

// public class Aligntotag extends Command {
//     private DriveTrain m_driveTrain;
//     /*private PhotonCamera leftcamera;
//     private PhotonCamera rightcamera;
//     private Pose2d camera1FieldPose;
//     private Pose2d camera2FieldPose;
//     private Vision vision;*/
//     private Transform3d camToRobot;
//     private double offsetDistance = 1;
//     private double offsetSign = 0.5;

//     private Vision m_vision;
//     private boolean onTrue;
//     private Supplier<Boolean> left1;
//     private Supplier<Boolean> left2;
//     private Supplier<Boolean> left3; 
//     private Supplier<Boolean> left4; 
//     private Supplier<Boolean> left5; 
//     private Supplier<Boolean> left6; 
//     private Supplier<Boolean> right1;
//     private Supplier<Boolean> right2;
//     private Supplier<Boolean> right3;
//     private Supplier<Boolean> right4;
//     private Supplier<Boolean> right5;
//     private Supplier<Boolean> right6;
//     Supplier<Boolean> falseSupplier = () -> false;

   
//     private PIDController xController = new PIDController(1.0, 0.0, 0.0);//要測才知道
//     private PIDController yController = new PIDController(1.0, 0.0, 0.0);//要測才知道
//     private PIDController thetaController = new PIDController(1.0, 0.0, 0.0);//要測才知道

//     // private final KalmanPoseEstimator kalmanEstimator = new KalmanPoseEstimator();
   
//      /*TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
//             Math.PI,   // max angular velocity (rad/s)
//             Math.PI    // max angular acceleration (rad/s^2)
//         );*/

//     private Pose2d targetPose = null;
//        // private HolonomicDriveController driveController;
    
// <<<<<<< Updated upstream
//     public Aligntotag(DriveTrain m_driveTrain, Vision m_vision//, 
//     //    Supplier<Boolean> left1, 
//     //    Supplier<Boolean> left2, 
//     //    Supplier<Boolean> left3, 
//     //    Supplier<Boolean> left4, 
//     //    Supplier<Boolean> left5, 
//     //    Supplier<Boolean> left6, 
//     //    Supplier<Boolean> right1,
//     //    Supplier<Boolean> right2,
//     //    Supplier<Boolean> right3,
//     //    Supplier<Boolean> right4,
//     //    Supplier<Boolean> right5,
//     //    Supplier<Boolean> right6
//     ) {
//         this.m_driveTrain = m_driveTrain;
//         this.m_vision = m_vision;
//         // this.left1 = left1;
//         // this.left2 = left2;
//         // this.left3 = left3;
//         // this.left4 = left4;
//         // this.left5 = left5;
//         // this.left6 = left6;
//         // this.right1 = right1;
//         // this.right2 = right2;
//         // this.right3 = right3;
//         // this.right4 = right4;
//         // this.right5 = right5;
//         // this.right6 = right6;
// =======
//     public Aligntotag(DriveTrain m_driveTrain, Vision m_vision, 
//        Supplier<Boolean> left1, 
//        Supplier<Boolean> left2, 
//        Supplier<Boolean> left3, 
//        Supplier<Boolean> left4, 
//        Supplier<Boolean> left5, 
//        Supplier<Boolean> left6, 
//        Supplier<Boolean> right1,
//        Supplier<Boolean> right2,
//        Supplier<Boolean> right3,
//        Supplier<Boolean> right4,
//        Supplier<Boolean> right5,
//        Supplier<Boolean> right6) {
//         this.m_driveTrain = m_driveTrain;
//         this.m_vision = m_vision;
//         this.left1 = left1;
//         this.left2 = left2;
//         this.left3 = left3;
//         this.left4 = left4;
//         this.left5 = left5;
//         this.left6 = left6;
//         this.right1 = right1;
//         this.right2 = right2;
//         this.right3 = right3;
//         this.right4 = right4;
//         this.right5 = right5;
//         this.right6 = right6;
// >>>>>>> Stashed changes
//         /*this.leftcamera = leftcamera;
//         this.rightcamera = rightcamera; 
//         this.vision = vision;
//         this.camera1FieldPose = camera1FieldPose;
//         this.camera2FieldPose = camera2FieldPose;
//         this.camToRobot = camToRobot;
//         this.targetPose = null; 
//         this.kalmanEstimator = new KalmanPoseEstimator();*/
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);
//         addRequirements(m_driveTrain, m_vision);
//     }        



//     @Override
//     public void execute() {
//         Pose2d robotPose = m_vision.getEstimatedPose();     
//         Pose2d closestTag = m_vision.getClosestAprilTagPose(robotPose);
//         if (targetPose == null) {
//             m_driveTrain.drive(new ChassisSpeeds());
//             return;
//         }

// <<<<<<< Updated upstream
//         if (targetPose == null) {
//             targetPose = m_vision.getClosestAprilTagPose(m_vision.getEstimatedPose());
//         }
        
//         // if(left1.get() || left2.get() || left3.get() || left4.get() || left5.get() || left6.get()){
//         //     offsetSign=1;
//         //     onTrue=true;
//         // }else if(right1.get() || right2.get() || right3.get() || right4.get() || right5.get() || right6.get()){
//         //     offsetSign=-1;
//         //     onTrue=true;
//         // }else{
//         //     onTrue=false;
//         // }
//         // if(onTrue){
// =======
//         if(left1.get() || left2.get() || left3.get() || left4.get() || left5.get() || left6.get()){
//             offsetSign=1;
//             onTrue=true;
//         }else if(right1.get() || right2.get() || right3.get() || right4.get() || right5.get() || right6.get()){
//             offsetSign=-1;
//             onTrue=true;
//         }else{
//             onTrue=false;
//         }
//         if(onTrue){
// >>>>>>> Stashed changes
//             Translation2d offset = new Translation2d(offsetDistance * offsetSign, 0).rotateBy(closestTag.getRotation());
//             Translation2d targetTranslation = closestTag.getTranslation().plus(offset);
//             targetPose = new Pose2d(targetTranslation, closestTag.getRotation());
//             double xSpeed = xController.calculate(robotPose.getX(), targetPose.getX());
//             double ySpeed = yController.calculate(robotPose.getY(), targetPose.getY());
//             double rotSpeed = thetaController.calculate(robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
//             xSpeed = clamp(xSpeed, -1.5, 1.5);//要測才知道
//             ySpeed = clamp(ySpeed, -1.5, 1.5);//要測才知道
//             rotSpeed = clamp(rotSpeed, -3.0, 3.0);//要測才知道
//             m_driveTrain.drive(new ChassisSpeeds( xSpeed, ySpeed, rotSpeed));
// <<<<<<< Updated upstream
//         //}else{
//             // m_driveTrain.drive();  手動的部分 但我忘了丟進來
//         //}
// =======
//         }else{
//             // m_driveTrain.drive();手動的部分 但我忘了丟進來
//         }
// >>>>>>> Stashed changes
        
            
        
        

//         /*Pose2d estimate1 = getRobotPoseFromCamera(leftcamera, camera1FieldPose);
//         Pose2d estimate2 = getRobotPoseFromCamera(rightcamera, camera2FieldPose);
        
//         if (estimate1 != null) kalmanEstimator.update(estimate1);
//         if (estimate2 != null) kalmanEstimator.update(estimate2);
        
//         Pose2d currentPose = kalmanEstimator.getEstimatedPose();*/

//         // Pose2d maybePose = m_vision.getEstimatedPose();
//         // if (maybePose.isEmpty()) {
//         //     m_driveTrain.drive(new ChassisSpeeds(0, 0, 0));
//         //     return;
//         // }
//         // kalmanEstimator.update(robotPose);
//         // robotPose = kalmanEstimator.getEstimatedPose();
        
//         // 如果沒有找到目標，則不進行對齊
//         //Pose2d closestTag = getClosestAprilTagPose(currentPose);
//         //if (closestTag == null) return; 
        
//         //Translation2d error = closestTag.getTranslation().minus(currentPose.getTranslation());
        

//         // kalmanEstimator.update(robotPose);
        
//         }

        

       
//             /**
//              * 將速度限制在指定範圍內
//              * @param value 要限制的值
//              * @param min 最小值
//              * @param max 最大值
//              * @return 限制後的值
//              */
        
//             private double clamp(double value, double min, double max) {
//                 return Math.max(min, Math.min(max, value));
//             }
        
//             @Override
//             public boolean isFinished() {
//                 //Pose2d target = getClosestAprilTagPose(kalmanEstimator.getEstimatedPose());
//                 // if (targetPose == null) return false;
//                 // Pose2d error = kalmanEstimator.getEstimatedPose().relativeTo(targetPose);
//                 // return error.getTranslation().getNorm() < 0.1 && Math.abs(error.getRotation().getDegrees()) < 3;
//                 return false;
//             }
        
//             @Override
//             public void end(boolean interrupted) {
//                 m_driveTrain.drive(new ChassisSpeeds());
//             }
        
//     //         static class KalmanPoseEstimator {
//     //             private Pose2d estimatedPose = new Pose2d();
//     //             private final Timer timer = new Timer();
        
//     //             public KalmanPoseEstimator() {
//     //                 timer.start();
//     //             }
        
//     //             public void reset(Pose2d estimate1) {
//     //                 // TODO Auto-generated method stub
//     //                 throw new UnsupportedOperationException("Unimplemented method 'reset'");
//     //             }
        
//     //             public void update(Pose2d measurement) {
//     //         estimatedPose = new Pose2d(
//     //             (estimatedPose.getX() + measurement.getX()) / 2.0,
//     //             (estimatedPose.getY() + measurement.getY()) / 2.0,
//     //             estimatedPose.getRotation().interpolate(measurement.getRotation(), 0.5)
//     //         );
//     //     }

//     //     public Pose2d getEstimatedPose() {
//     //         return estimatedPose;
//     //     }
//     // }
//     /*private Pose2d getRobotPoseFromCamera(PhotonCamera camera, Pose2d camFieldPose) {
//                 var result = camera.getLatestResult();
//                 if (!result.hasTargets()) return null;
        
//                 PhotonTrackedTarget target = result.getBestTarget();
//                 Transform3d camToTarget = target.getBestCameraToTarget();
        
//                 Pose3d fieldToTarget = new Pose3d(camFieldPose).transformBy(camToTarget);
//                 Pose3d fieldToRobot = fieldToTarget.transformBy(camToRobot);


//                  Rotation3d rot = fieldToRobot.getRotation();
//                     double yawRadians = rot.getZ();
        
//                 return new Pose2d(
//                     fieldToRobot.getX(),
//                     fieldToRobot.getY(),
//                     new Rotation2d(yawRadians)
//                 );//丟板上面
//             }*/
        
//             // private Pose2d getClosestAprilTagPose(Pose2d currentPose) {
//             //     List<Pose2d> allTags = m_driveTrain.getFieldAprilTagPoses(); // 使用者應提供場地所有 AprilTag Pose
//             //     Optional<Pose2d> closest = allTags.stream()
//             //         .min(Comparator.comparingDouble(p -> p.getTranslation().getDistance(currentPose.getTranslation())));
//             //     return closest.orElse(null);
//             // }
// }

