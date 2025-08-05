package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.*;
import frc.robot.Tables;
import frc.robot.constants.ConsCamera;
import frc.robot.constants.ConsSwerve;
import frc.robot.subsystems.DriveTrain;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {
    private final PhotonCamera leftCamera = new PhotonCamera("ap2");
    private final PhotonCamera rightCamera = new PhotonCamera("ap1");
    private final Tables yolo = new Tables("coral");

    private final AprilTagFieldLayout tagLayout = ConsCamera.kTagLayout;
    private final DriveTrain m_driveTrain;

    private Pose2d lastPose = new Pose2d();
    private double lastUpdateTime = 0;

    private static final double MIN_UPDATE_INTERVAL = 0.5;
    private static final double MAX_POSITION_JUMP = 1.0;
    private static final double MAX_ANGLE_JUMP = 20.0;
    private static final double MAX_AMBIGUITY = 0.3;
    private static final double FILTER_ALPHA = 0.6;


    public Vision(DriveTrain dt) {
        this.m_driveTrain = dt;
    }

    @Override
    public void periodic() {
        PoseEstimateWithAmbiguity left = getPoseFromMultipleTags(leftCamera, ConsCamera.kCameraToRobotLeft);
        PoseEstimateWithAmbiguity right = getPoseFromMultipleTags(rightCamera, ConsCamera.kCameraToRobotRight);
        PoseEstimateWithAmbiguity best = selectBetterPose(left, right);
    
        if (best == null) {
            // Multi-tag 無法取得有效位置，使用 single-tag fallback
            left = getPoseFromSingleTag(leftCamera, ConsCamera.kCameraToRobotLeft);
            right = getPoseFromSingleTag(rightCamera, ConsCamera.kCameraToRobotRight);
            best = selectBetterPose(left, right);
        }
    
        if (best != null) {
            tryUpdatePose(best.pose, best.ambiguity, best.source);
        } else {
            SmartDashboard.putString("VisionUpdate", "❌ No valid tag");
        }
    }
    private PoseEstimateWithAmbiguity getPoseFromSingleTag(PhotonCamera camera, Transform3d cameraToRobot) {
        PhotonPipelineResult result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        if (targets.isEmpty()) return null;

        targets.sort(Comparator.comparingDouble(PhotonTrackedTarget::getPoseAmbiguity));
        for (PhotonTrackedTarget target : targets) {
            double amb = target.getPoseAmbiguity();
            if (amb < 0 || amb > MAX_AMBIGUITY) continue;

            var tagPoseOpt = tagLayout.getTagPose(target.getFiducialId());
            if (tagPoseOpt.isEmpty()) continue;

            var tagToCamera = target.getBestCameraToTarget().inverse();
            if (tagToCamera.getTranslation().getNorm() < 1e-6) continue;

            var cameraPose = tagPoseOpt.get().transformBy(tagToCamera);
            var robotPose = cameraPose.transformBy(cameraToRobot);
            return new PoseEstimateWithAmbiguity(robotPose.toPose2d(), amb, camera.getName(), 1);
        }
        return null;
    }

    private PoseEstimateWithAmbiguity getPoseFromMultipleTags(PhotonCamera camera, Transform3d cameraToRobot) {
        PhotonPipelineResult result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        if (targets.isEmpty()) return null;

        List<Pose2d> poses = new ArrayList<Pose2d>();
        double totalAmbiguity = 0;

        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPoseOpt = tagLayout.getTagPose(target.getFiducialId());
            if (tagPoseOpt.isEmpty()) continue;

            Transform3d tagToCamera = target.getBestCameraToTarget().inverse();
            Pose3d cameraPose = tagPoseOpt.get().transformBy(tagToCamera);
            Pose3d robotPose = cameraPose.transformBy(cameraToRobot);
            poses.add(robotPose.toPose2d());
            totalAmbiguity += target.getPoseAmbiguity();
        }

        if (poses.isEmpty()) return null;

        // 平均位置
        Translation2d avgPos = new Translation2d();
        double sumSin = 0;
        double sumCos = 0;

        for (Pose2d p : poses) {
            avgPos = avgPos.plus(p.getTranslation());
            sumSin += Math.sin(p.getRotation().getRadians());
            sumCos += Math.cos(p.getRotation().getRadians());
        }

        avgPos = avgPos.div(poses.size());
        Rotation2d avgRot = new Rotation2d(Math.atan2(sumSin, sumCos));
        Pose2d fusedPose = new Pose2d(avgPos, avgRot);
        double avgAmbiguity = totalAmbiguity / poses.size();

        return new PoseEstimateWithAmbiguity(fusedPose, avgAmbiguity, camera.getName(), poses.size());
    }

    private PoseEstimateWithAmbiguity selectBetterPose(PoseEstimateWithAmbiguity a, PoseEstimateWithAmbiguity b) {
        if (a == null && b == null) return null;
        if (a == null) return b;
        if (b == null) return a;

        if (a.tagCount >= 2 && b.tagCount < 2) return a;
        if (b.tagCount >= 2 && a.tagCount < 2) return b;

        return a.ambiguity <= b.ambiguity ? a : b;
    }

    private boolean hasUpdatedOnce = false;

private void tryUpdatePose(Pose2d newPose, double ambiguity, String source) {
    double now = Timer.getFPGATimestamp();

    if (!hasUpdatedOnce) {
        // 第一次 → 不檢查差距，直接更新
        lastPose = newPose;
        lastUpdateTime = now;
        hasUpdatedOnce = true;

        m_driveTrain.updatePose(newPose);//(newPose.getX(), newPose.getY(), m_driveTrain.getDiretion().getDegrees());

        // SmartDashboard.putString("VisionUpdate", "✅ First-Time Pose Set from " + source);
        return;
    }

    // if (now - lastUpdateTime < MIN_UPDATE_INTERVAL) {
    //     SmartDashboard.putString("VisionUpdate", "⏳ Too soon");
    //     return;
    // }

    double posDiff = newPose.getTranslation().getDistance(lastPose.getTranslation());
    double angleDiff = Math.abs(newPose.getRotation().minus(lastPose.getRotation()).getDegrees());

    if (ambiguity > MAX_AMBIGUITY) {
        SmartDashboard.putString("VisionUpdate", "❌ Ambiguity too high");
        return;
    }
    
    if (posDiff > MAX_POSITION_JUMP || angleDiff > MAX_ANGLE_JUMP) {
        SmartDashboard.putString("VisionUpdate", "❌ Jump too large");
        return;
    }

    lastPose = newPose;
    lastUpdateTime = now;

    m_driveTrain.updatePose(newPose.getX(), newPose.getY(), m_driveTrain.getDiretion().getDegrees());
    // SmartDashboard.putString("VisionUpdate", "✅ Pose Updated from " + source);
}


    public Pose2d getVisionPose() {
        return lastPose;
    }

    private static class PoseEstimateWithAmbiguity {
        public final Pose2d pose;
        public final double ambiguity;
        public final String source;
        public final int tagCount;

        public PoseEstimateWithAmbiguity(Pose2d pose, double ambiguity, String source, int tagCount) {
            this.pose = pose;
            this.ambiguity = ambiguity;
            this.source = source;
            this.tagCount = tagCount;
        }
    }
}