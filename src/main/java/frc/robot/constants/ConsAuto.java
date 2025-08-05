package frc.robot.constants;

import java.util.EnumMap;
import java.util.Map;

public class ConsAuto {
  public enum PositionName {
    r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12
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

    BLUE_POSITIONS.put(PositionName.r1, new Position(3.097252789383869, 4.238601838605371, 180));
    BLUE_POSITIONS.put(PositionName.r2, new Position(3.6220668706568615, 5.208761711783277, -60));//3.64,5.189
    BLUE_POSITIONS.put(PositionName.r3, new Position(3.9414385701436214, 5.366222037240583, 120));//3.925,5.353
    BLUE_POSITIONS.put(PositionName.r4, new Position(5.089007434526264, 5.318804374579443, 60));//
    BLUE_POSITIONS.put(PositionName.r5, new Position(5.408497596481436, 5.153439760905154, 60));//
    BLUE_POSITIONS.put(PositionName.r6, new Position(5.891864991097945, 4.171499249538577, 0));//
    BLUE_POSITIONS.put(PositionName.r7, new Position(5.927356837113303, 3.8370305156669398, 0));//
    BLUE_POSITIONS.put(PositionName.r8, new Position(5.300692371365355, 2.822505803592738, -60));
    BLUE_POSITIONS.put(PositionName.r9, new Position(4.994593666574269, 2.709344111193596, -60));
    BLUE_POSITIONS.put(PositionName.r10, new Position(3.8654971188190186, 2.8192787286917196, -120));
    BLUE_POSITIONS.put(PositionName.r11, new Position(3.6552134474652083, 2.937000141763921, -120));//
    BLUE_POSITIONS.put(PositionName.r12, new Position(3.1362454606429506, 3.7788990186050646, 180));//
  }

  public static Position pos(String name) {
    return getPosition(PositionName.valueOf(name));
}

}
