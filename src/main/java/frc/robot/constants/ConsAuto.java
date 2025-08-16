package frc.robot.constants;

import java.util.EnumMap;
import java.util.Map;

public class ConsAuto {
  public enum PositionName {
    r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, B1, B2, B3
  }

  public static class Position {
    public final double x, y, z;

    public Position(double x, double y, double z) {
      this.x = x;
      this.y = y;
      this.z = z;
    }
  }

  // 紅藍方設定（預設為紅方）
  private static boolean isRedAlliance = true;

  // 提供外部設定紅藍方的方法
  public static void setAllianceColor(boolean isRed) {
    isRedAlliance = isRed;
  }

  // 外部用來取得位置的方法（已內部判斷紅藍方）
  public static Position getPosition(PositionName name) {
    return isRedAlliance ? RED_POSITIONS.get(name) : BLUE_POSITIONS.get(name);
  }

  public static final Map<PositionName, Position> RED_POSITIONS = new EnumMap<>(PositionName.class);
  public static final Map<PositionName, Position> BLUE_POSITIONS = new EnumMap<>(PositionName.class);

  static {
    RED_POSITIONS.put(PositionName.r1, new Position(14.505, 3.859, 0));
    RED_POSITIONS.put(PositionName.r2, new Position(13.933, 2.858, -60));
    RED_POSITIONS.put(PositionName.r3, new Position(13.632, 2.706, -60));
    RED_POSITIONS.put(PositionName.r4, new Position(12.488, 2.697, -120));
    RED_POSITIONS.put(PositionName.r5, new Position(12.201, 2.868, -120));
    RED_POSITIONS.put(PositionName.r6, new Position(11.624, 3.860, 180));
    RED_POSITIONS.put(PositionName.r7, new Position(11.624, 4.188, 180));
    RED_POSITIONS.put(PositionName.r8, new Position(12.199, 5.189, 120));
    RED_POSITIONS.put(PositionName.r9, new Position(12.5, 5.353, 120));
    RED_POSITIONS.put(PositionName.r10, new Position(13.651, 5.378, 60));
    RED_POSITIONS.put(PositionName.r11, new Position(13.941, 5.214, 60));
    RED_POSITIONS.put(PositionName.r12, new Position(14.5, 4.193, 0));
    RED_POSITIONS.put(PositionName.B1, new Position(14.5, 4.193, 0));
    RED_POSITIONS.put(PositionName.B2, new Position(14.5, 4.193, 0));
    RED_POSITIONS.put(PositionName.B3, new Position(14.5, 4.193, 0));

    BLUE_POSITIONS.put(PositionName.r1, new Position(3.052, 4.186, 180));
    BLUE_POSITIONS.put(PositionName.r2, new Position(3.640, 5.189, 120));
    BLUE_POSITIONS.put(PositionName.r3, new Position(3.925, 5.353, 120));
    BLUE_POSITIONS.put(PositionName.r4, new Position(5.05, 5.351, 60));
    BLUE_POSITIONS.put(PositionName.r5, new Position(5.348, 5.179, 60));
    BLUE_POSITIONS.put(PositionName.r6, new Position(5.93, 4.186, 0));
    BLUE_POSITIONS.put(PositionName.r7, new Position(5.93, 3.857, 0));
    BLUE_POSITIONS.put(PositionName.r8, new Position(5.341, 2.87, -60));
    BLUE_POSITIONS.put(PositionName.r9, new Position(5.06, 2.702, -60));
    BLUE_POSITIONS.put(PositionName.r10, new Position(3.941, 2.691, -120));
    BLUE_POSITIONS.put(PositionName.r11, new Position(3.634, 2.872, -120));
    BLUE_POSITIONS.put(PositionName.r12, new Position(3.052, 3.854, 180));
    BLUE_POSITIONS.put(PositionName.B1, new Position(1, 2.194, 180));//
    BLUE_POSITIONS.put(PositionName.B2, new Position(2.063, 4.040, 180));//
    BLUE_POSITIONS.put(PositionName.B3, new Position(2.238, 2.194, 180));//

  }

  public static Position pos(String name) {
    return getPosition(PositionName.valueOf(name));
  }

}
