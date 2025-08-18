package frc.robot.constants;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class ConsAuto {
  public enum PositionName {
    r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, B1, B2, B3,Rr
  }

  // 預設紅方，開場自動同步一次
  private static boolean isRedAlliance = true;
  static { syncAllianceFromDS(); }

  public static void setAllianceColor(boolean isRed) {
    isRedAlliance = isRed;
  }

  /** 從 Driver Station 取得目前隊伍顏色，沒有資料時預設紅方 */
  public static void syncAllianceFromDS() {
    var alliance = DriverStation.getAlliance();
    isRedAlliance = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : true;
  }

  public static Pose2d getPosition(PositionName name) {
    return isRedAlliance ? RED_POSITIONS.get(name) : BLUE_POSITIONS.get(name);
  }

  public static Pose2d pos(PositionName name) {
    return getPosition(name);
  }

  public static final Map<PositionName, Pose2d> RED_POSITIONS = new EnumMap<>(PositionName.class);
  public static final Map<PositionName, Pose2d> BLUE_POSITIONS = new EnumMap<>(PositionName.class);

  static {
    // RED
    RED_POSITIONS.put(PositionName.r1,  new Pose2d(14.505, 3.859, Rotation2d.fromDegrees(0)));
    RED_POSITIONS.put(PositionName.r2,  new Pose2d(13.933, 2.858, Rotation2d.fromDegrees(-60)));
    RED_POSITIONS.put(PositionName.r3,  new Pose2d(13.632, 2.706, Rotation2d.fromDegrees(-60)));
    RED_POSITIONS.put(PositionName.r4,  new Pose2d(12.488, 2.697, Rotation2d.fromDegrees(-120)));
    RED_POSITIONS.put(PositionName.r5,  new Pose2d(12.201, 2.868, Rotation2d.fromDegrees(-120)));
    RED_POSITIONS.put(PositionName.r6,  new Pose2d(11.624, 3.860, Rotation2d.fromDegrees(180)));
    RED_POSITIONS.put(PositionName.r7,  new Pose2d(11.624, 4.188, Rotation2d.fromDegrees(180)));
    RED_POSITIONS.put(PositionName.r8,  new Pose2d(12.199, 5.189, Rotation2d.fromDegrees(120)));
    RED_POSITIONS.put(PositionName.r9,  new Pose2d(12.5,   5.353, Rotation2d.fromDegrees(120)));
    RED_POSITIONS.put(PositionName.r10, new Pose2d(13.651, 5.378, Rotation2d.fromDegrees(60)));
    RED_POSITIONS.put(PositionName.r11, new Pose2d(13.941, 5.214, Rotation2d.fromDegrees(60)));
    RED_POSITIONS.put(PositionName.r12, new Pose2d(14.5,   4.193, Rotation2d.fromDegrees(0)));
    RED_POSITIONS.put(PositionName.B1,  new Pose2d(14.5,   4.193, Rotation2d.fromDegrees(0)));
    RED_POSITIONS.put(PositionName.B2,  new Pose2d(14.5,   4.193, Rotation2d.fromDegrees(0)));
    RED_POSITIONS.put(PositionName.B3,  new Pose2d(14.5,   4.193, Rotation2d.fromDegrees(0)));
    BLUE_POSITIONS.put(PositionName.B3,  new Pose2d(3.69, 2.609, Rotation2d.fromDegrees(180)));

    // BLUE
    BLUE_POSITIONS.put(PositionName.r1,  new Pose2d(3.052, 4.186, Rotation2d.fromDegrees(180)));
    BLUE_POSITIONS.put(PositionName.r2,  new Pose2d(3.640, 5.189, Rotation2d.fromDegrees(120)));
    BLUE_POSITIONS.put(PositionName.r3,  new Pose2d(3.925, 5.353, Rotation2d.fromDegrees(120)));
    BLUE_POSITIONS.put(PositionName.r4,  new Pose2d(5.05,  5.351, Rotation2d.fromDegrees(60)));
    BLUE_POSITIONS.put(PositionName.r5,  new Pose2d(5.348, 5.179, Rotation2d.fromDegrees(60)));
    BLUE_POSITIONS.put(PositionName.r6,  new Pose2d(5.93,  4.186, Rotation2d.fromDegrees(0)));
    BLUE_POSITIONS.put(PositionName.r7,  new Pose2d(5.93,  3.857, Rotation2d.fromDegrees(0)));
    BLUE_POSITIONS.put(PositionName.r8,  new Pose2d(5.341, 2.87,  Rotation2d.fromDegrees(-60)));
    BLUE_POSITIONS.put(PositionName.r9,  new Pose2d(4.994409931624252,  2.639007039296366, Rotation2d.fromDegrees(-60)));//
    BLUE_POSITIONS.put(PositionName.r10, new Pose2d(3.847, 2.72, Rotation2d.fromDegrees(-120)));//
    BLUE_POSITIONS.put(PositionName.r11, new Pose2d(3.634, 2.872, Rotation2d.fromDegrees(-120)));
    BLUE_POSITIONS.put(PositionName.r12, new Pose2d(3.052, 3.854, Rotation2d.fromDegrees(180)));
    BLUE_POSITIONS.put(PositionName.B1,  new Pose2d(1,     2.194, Rotation2d.fromDegrees(180)));
    BLUE_POSITIONS.put(PositionName.B2,  new Pose2d(2.063, 4.040, Rotation2d.fromDegrees(180)));
    BLUE_POSITIONS.put(PositionName.B3,  new Pose2d(2.238, 2.194, Rotation2d.fromDegrees(180)));
    BLUE_POSITIONS.put(PositionName.Rr,  new Pose2d(3.69, 2.609, Rotation2d.fromDegrees(-120)));

  }
}
