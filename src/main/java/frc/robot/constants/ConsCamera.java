package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * 座標系（WPILib）：+X 前、+Y 左、+Z 上
 * Rotation3d(rollX, pitchY, yawZ) 依序繞 X/Y/Z，單位弧度。
 *
 * 原點：車子地面正中心。
 *
 * 相機為「朝車頭反方向」安裝，故在 yaw 上加 180°。
 * 位置數值完全沿用你的原始值，不動。
 *
 * 提供兩種方向：
 *  - kRobotToCameraLeft/Right  ：機器人 → 相機（PhotonPoseEstimator 需要）
 *  - kCameraToRobotLeft/Right  ：相機 → 機器人（相容用途）
 */
public final class ConsCamera {
  private ConsCamera() {}

  // ===================== 相機 → 機器人（保留你的位置，yaw 加 180°） =====================
  // 左相機：位置 (-0.216, -0.139, 0.173) m；姿態 roll=0, pitch=+15°, yaw=-5° + 180° = 175°
  public static final Transform3d kCameraToRobotLeft =
      new Transform3d(
          new Translation3d(-0.216, -0.139, 0.173),
          new Rotation3d(
              0.0,
              Math.toRadians(15.0),
              Math.toRadians(184.25)
          )
      );

  // 右相機：位置 (-0.197058, +0.288, 0.173) m；姿態 roll=0, pitch=+15°, yaw=+15° + 180° = 195°
  public static final Transform3d kCameraToRobotRight =
      new Transform3d(
          new Translation3d(-0.197058, 0.288, 0.173),//-0.197058, 0.288, 0.173
          new Rotation3d(
              0.0,
              Math.toRadians(15.0),//15
              Math.toRadians(166.142)   // 等同 -165°
          )
      );

  // ===================== 機器人 → 相機（給 PhotonPoseEstimator） =====================
  public static final Transform3d kRobotToCameraLeft  = kCameraToRobotLeft.inverse();
  public static final Transform3d kRobotToCameraRight = kCameraToRobotRight.inverse();

  // ===================== 2025 場地 Layout =====================
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
}
