// package frc.robot.subsystems;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.*;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.constants.ConsCamera;
// import frc.robot.Tables;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import java.util.*;

// public class VirtualPose extends SubsystemBase {
//     private final PhotonCamera leftCam = new PhotonCamera("LeftCam");
//     private final PhotonCamera rightCam = new PhotonCamera("RightCam");
//     private final Tables backCam = new Tables("BackCam");

//     private final Transform3d leftCamToRobot = ConsCamera.kCameraToRobotLeft;
//     private final Transform3d rightCamToRobot = ConsCamera.kCameraToRobotRight;
//     private final AprilTagFieldLayout tagLayout;

//     private final DriveTrain m_driveTrain;
//     private SwerveDrivePoseEstimator swervePoseEstimator;

//     private Pose2d lastPose = new Pose2d();
//     private double lastUpdateTime = 0;
//     private boolean hasUpdatedOnce = false;

//     private static final double Z_TOLERANCE = 0.5;

//     public VirtualPose(DriveTrain driveTrain) {
//         this.m_driveTrain = driveTrain;
//         this.tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
//     }

//     public void setPoseEstimator(SwerveDrivePoseEstimator estimator) {
//         this.swervePoseEstimator = estimator;
//     }

//     public double getTargetDistance() {
//         return backCam.getX();
//     }

//     public double getTargetAngle() {
//         return backCam.getY();
//     }

//     @Override
//     public void periodic() {
//         PoseEstimateWithAmbiguity leftMulti = getPoseFromMultipleTags(leftCam, leftCamToRobot);
//         PoseEstimateWithAmbiguity rightMulti = getPoseFromMultipleTags(rightCam, rightCamToRobot);

//         PoseEstimateWithAmbiguity bestMulti = selectBetterMultiTagPose(leftMulti, rightMulti);

//         if (bestMulti != null) {
//             tryUpdatePose(bestMulti.pose, bestMulti.ambiguity, bestMulti.source + " (Multi)");
//             return;
//         }

//         // fallback to single-tag mode
//         PoseEstimateWithAmbiguity leftSingle = getPoseFromSingleTag(leftCam, leftCamToRobot);
//         PoseEstimateWithAmbiguity rightSingle = getPoseFromSingleTag(rightCam, rightCamToRobot);
//         PoseEstimateWithAmbiguity bestSingle = selectBetterPose(leftSingle, rightSingle);

//         if (bestSingle != null) {
//             tryUpdatePose(bestSingle.pose, bestSingle.ambiguity, bestSingle.source + " (Single)");
//         }
//     }

//     private PoseEstimateWithAmbiguity getPoseFromMultipleTags(PhotonCamera camera, Transform3d cameraToRobot) {
//         PhotonPipelineResult result = camera.getLatestResult();
//         List<PhotonTrackedTarget> targets = result.getTargets();
//         if (targets.isEmpty()) return null;

//         List<Pose2d> poses = new ArrayList<>();
//         double totalAmbiguity = 0;

//         for (PhotonTrackedTarget target : targets) {
//             Optional<Pose3d> tagPoseOpt = tagLayout.getTagPose(target.getFiducialId());
//             if (tagPoseOpt.isEmpty()) continue;

//             Transform3d tagToCamera = target.getBestCameraToTarget().inverse();
//             Pose3d cameraPose = tagPoseOpt.get().transformBy(tagToCamera);
//             Pose3d robotPose = cameraPose.transformBy(cameraToRobot);

//             poses.add(robotPose.toPose2d());
//             totalAmbiguity += target.getPoseAmbiguity();
//         }

//         if (poses.isEmpty()) return null;

//         Translation2d avgPos = new Translation2d();
//         double sumSin = 0, sumCos = 0;

//         for (Pose2d pose : poses) {
//             avgPos = avgPos.plus(pose.getTranslation());
//             sumSin += Math.sin(pose.getRotation().getRadians());
//             sumCos += Math.cos(pose.getRotation().getRadians());
//         }

//         avgPos = avgPos.div(poses.size());
//         Rotation2d avgRot = new Rotation2d(Math.atan2(sumSin, sumCos));

//         Pose2d fusedPose = new Pose2d(avgPos, avgRot);
//         double avgAmbiguity = totalAmbiguity / poses.size();

//         return new PoseEstimateWithAmbiguity(fusedPose, avgAmbiguity, camera.getName(), poses.size());
//     }

//     private PoseEstimateWithAmbiguity getPoseFromSingleTag(PhotonCamera camera, Transform3d cameraToRobot) {
//         PhotonPipelineResult result = camera.getLatestResult();
//         List<PhotonTrackedTarget> targets = result.getTargets();
//         if (targets.isEmpty()) return null;

//         for (PhotonTrackedTarget target : targets) {
//             Optional<Pose3d> tagPoseOpt = tagLayout.getTagPose(target.getFiducialId());
//             if (tagPoseOpt.isEmpty()) continue;

//             Transform3d tagToCamera = target.getBestCameraToTarget().inverse();
//             Pose3d cameraPose = tagPoseOpt.get().transformBy(tagToCamera);
//             Pose3d robotPose = cameraPose.transformBy(cameraToRobot);

//             if (Math.abs(robotPose.getZ()) > Z_TOLERANCE) continue;

//             return new PoseEstimateWithAmbiguity(robotPose.toPose2d(), target.getPoseAmbiguity(), camera.getName(), 1);
//         }

//         return null;
//     }

//     private PoseEstimateWithAmbiguity selectBetterPose(PoseEstimateWithAmbiguity a, PoseEstimateWithAmbiguity b) {
//         if (a == null && b == null) return null;
//         if (a == null) return b;
//         if (b == null) return a;
//         return a.ambiguity <= b.ambiguity ? a : b;
//     }

//     private PoseEstimateWithAmbiguity selectBetterMultiTagPose(PoseEstimateWithAmbiguity a, PoseEstimateWithAmbiguity b) {
//         boolean aValid = a != null && a.tagCount >= 2;
//         boolean bValid = b != null && b.tagCount >= 2;

//         if (aValid && !bValid) return a;
//         if (bValid && !aValid) return b;
//         if (aValid && bValid) return a.ambiguity <= b.ambiguity ? a : b;

//         return null;
//     }

//     private void tryUpdatePose(Pose2d newPose, double ambiguity, String source) {
//         double now = Timer.getFPGATimestamp();

//         if (!hasUpdatedOnce) {
//             hasUpdatedOnce = true;
//             lastPose = newPose;
//             lastUpdateTime = now;

//             m_driveTrain.updatePose(newPose.getX(), newPose.getY(), newPose.getRotation().getDegrees());
//             if (swervePoseEstimator != null)
//                 swervePoseEstimator.resetPosition(m_driveTrain.getDiretion(), m_driveTrain.getModulePositions(), newPose);
//             return;
//         }

//         // 可加：差異過大或 ambiguity 太高時跳過
//         // double posDiff = newPose.getTranslation().getDistance(lastPose.getTranslation());
//         // double angleDiff = Math.abs(newPose.getRotation().minus(lastPose.getRotation()).getDegrees());

//         lastPose = newPose;
//         lastUpdateTime = now;

//         m_driveTrain.updatePose(newPose.getX(), newPose.getY(), newPose.getRotation().getDegrees());
//         if (swervePoseEstimator != null)
//             swervePoseEstimator.addVisionMeasurement(newPose, now);

//         // SmartDashboard.putString("VisionUpdate", "✅ Updated from " + source);
//     }

//     private static class PoseEstimateWithAmbiguity {
//         public final Pose2d pose;
//         public final double ambiguity;
//         public final String source;
//         public final int tagCount;

//         public PoseEstimateWithAmbiguity(Pose2d pose, double ambiguity, String source, int tagCount) {
//             this.pose = pose;
//             this.ambiguity = ambiguity;
//             this.source = source;
//             this.tagCount = tagCount;
//         }
//     }
// }


