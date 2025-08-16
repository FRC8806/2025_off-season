package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

/** PhotonVision 2025.2.7 相容版 */
public class Vision extends SubsystemBase {
  // 跟 UI 名稱一致
  private final PhotonCamera leftCamera  = new PhotonCamera("ap2");
  private final PhotonCamera rightCamera = new PhotonCamera("ap1");

  // 你的常數
  private final AprilTagFieldLayout tagLayout = frc.robot.constants.ConsCamera.kTagLayout;
  private final Transform3d camToRobotLeft    = frc.robot.constants.ConsCamera.kCameraToRobotLeft;
  private final Transform3d camToRobotRight   = frc.robot.constants.ConsCamera.kCameraToRobotRight;

  private final DriveTrain driveTrain;

  // 舊 API：建構子只有 (layout, strategy, robotToCamera)
  private final PhotonPoseEstimator leftEstimator;
  private final PhotonPoseEstimator rightEstimator;

  // Dashboard keys
  private static final String KEY_ENABLE = "Vision/Enable";
  private static final String KEY_LAST   = "Vision/Last";

  // 開機/切自動保護
  private static final double ENABLE_DELAY_S = 1.0;
  private double enabledAt = -1.0;

  public Vision(DriveTrain dt) {
    this.driveTrain = dt;

    // Robot->Camera = (Camera->Robot)^-1
    Transform3d robotToCameraLeft  = camToRobotLeft.inverse();
    Transform3d robotToCameraRight = camToRobotRight.inverse();

    // 建立兩個 estimator，各自用自己的 Robot->Camera
    leftEstimator = new PhotonPoseEstimator(
        tagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCameraLeft
    );
    rightEstimator = new PhotonPoseEstimator(
        tagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCameraRight
    );

    // 只有 multi-tag 失敗時的退路策略，舊版名稱是 setMultiTagFallbackStrategy
    leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    SmartDashboard.putBoolean(KEY_ENABLE, true);
    SmartDashboard.putString(KEY_LAST, "none");
  }

  @Override
  public void periodic() {
    if (!SmartDashboard.getBoolean(KEY_ENABLE, true)) return;

    double now = Timer.getFPGATimestamp();
    if (enabledAt < 0) { enabledAt = now; return; }
    if (now - enabledAt < ENABLE_DELAY_S) return;

    // 參考姿態有助收斂（不論你選哪種策略都不壞事）
    Pose3d ref = new Pose3d(driveTrain.getPose());
    leftEstimator.setReferencePose(ref);
    rightEstimator.setReferencePose(ref);

    pushIfGood(leftCamera, leftEstimator);
    pushIfGood(rightCamera, rightEstimator);
  }

  private void pushIfGood(PhotonCamera cam, PhotonPoseEstimator estimator) {
    PhotonPipelineResult result = cam.getLatestResult();
    if (!result.hasTargets()) return;

    // 用帶 result 的 update，保留影像時間戳
    Optional<EstimatedRobotPose> opt = estimator.update(result);
    if (opt.isEmpty()) return;

    EstimatedRobotPose erp = opt.get();

    // 估品質：標籤數、平均 ambiguity、平均距離
    int tagCount = erp.targetsUsed.size();
    double avgAmb = 0.0, avgDist = 0.0;
    for (var t : erp.targetsUsed) {
      double amb = t.getPoseAmbiguity();
      if (amb >= 0) avgAmb += amb;
      avgDist += t.getBestCameraToTarget().getTranslation().getNorm();
    }
    if (tagCount > 0) { avgAmb /= tagCount; avgDist /= tagCount; }

    // 粗濾：單標且模糊或太遠就丟掉
    if (tagCount == 1 && (avgAmb > 0.30 || avgDist > 6.0)) {
      SmartDashboard.putString(KEY_LAST, cam.getName() + " drop(single, amb=" + fmt(avgAmb) + ", d=" + fmt(avgDist) + ")");
      return;
    }

    double ts = result.getTimestampSeconds(); // 影像時間
    VisionStdDevs std = VisionStdDevs.compute(tagCount, avgAmb, avgDist);

    Pose2d pose2d = erp.estimatedPose.toPose2d();
    driveTrain.addVisionMeasurement(pose2d, ts, std);

    SmartDashboard.putString(
        KEY_LAST,
        cam.getName() + " ok tags=" + tagCount +
        " amb=" + fmt(avgAmb) + " d=" + fmt(avgDist) +
        " -> [" + fmt(pose2d.getX()) + "," + fmt(pose2d.getY()) + "," +
        fmt(pose2d.getRotation().getDegrees()) + "]"
    );
  }

  private static String fmt(double v) { return String.format("%.3f", v); }

  /** 視覺量測標準差：依 tag 數、ambiguity、距離動態調整 */
  public static class VisionStdDevs {
    public final double stdX;         // m
    public final double stdY;         // m
    public final double stdThetaRad;  // rad

    private VisionStdDevs(double sx, double sy, double stRad) {
      this.stdX = sx; this.stdY = sy; this.stdThetaRad = stRad;
    }

    public static VisionStdDevs compute(int tagCount, double avgAmbiguity, double avgDistance) {
      double sx = 0.05, sy = 0.05, st = Math.toRadians(2.0); // 多標近距離基底
      if (tagCount <= 1) { sx *= 3.0; sy *= 3.0; st *= 2.0; }       // 單標放大

      double amb = clamp(avgAmbiguity, 0.0, 0.6);
      double ambScale = 1.0 + 2.0 * map(amb, 0.10, 0.35, 0.0, 1.0);
      sx *= ambScale; sy *= ambScale; st *= ambScale;

      double d = clamp(avgDistance, 0.5, 10.0);
      double dScale = 1.0 + 2.0 * map(d, 3.0, 8.0, 0.0, 1.0);
      sx *= dScale; sy *= dScale; st *= dScale;

      sx = clamp(sx, 0.03, 0.50);
      sy = clamp(sy, 0.03, 0.50);
      st = clamp(st, Math.toRadians(1.0), Math.toRadians(10.0));
      return new VisionStdDevs(sx, sy, st);
    }

    private static double clamp(double v, double lo, double hi) {
      return Math.max(lo, Math.min(hi, v));
    }
    private static double map(double v, double inLo, double inHi, double outLo, double outHi) {
      double t = (v - inLo) / (inHi - inLo + 1e-9);
      t = clamp(t, 0.0, 1.0);
      return outLo + (outHi - outLo) * t;
    }
  }
}
