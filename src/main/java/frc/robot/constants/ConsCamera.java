package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class ConsCamera {

  // 相機到機器人中心的 Transform（根據你們機構要調）
  // 左相機：左前方 30 公分，高 40 公分，微向外偏轉
  public static final Transform3d kCameraToRobotLeft =
      new Transform3d(
          new Translation3d(-0.216, -0.139, 0.173),   // x, y, z (公尺)// -0.195781, 0.284422, 0.72465
          new Rotation3d(0,Math.toRadians(15) , Math.toRadians(-5)) // pitch, yaw, roll (radian)
      );

  // 右相機：右前方 30 公分，高 40 公分，微向內偏轉
  public static final Transform3d kCameraToRobotRight =
      new Transform3d(
          new Translation3d(-0.197058, 0.288, 0.173),//-0.226915, -0.139796, 0.143209
          new Rotation3d(0, Math.toRadians(15), Math.toRadians(15))
      );

  // 場地的 AprilTag 布局：2025 Reefscape 焊接場地版本
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
}


