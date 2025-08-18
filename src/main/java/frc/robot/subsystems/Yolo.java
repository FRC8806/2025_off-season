package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 功能：
 *  1) 從 NetworkTables 表 "coral" 讀取鏡頭座標（支援數字/字串、像素/正規化）。
 *  2) 提供 createVisionDriveXY(DriveTrain) 產生指令，用 X 控轉向、Y 控前進。
 *
 * 座標規則：
 *  - 正規化模式：xc/yc ∈ [-1,1]，原點在影像中心；x 右正左負；y 上正下負。
 *  - 若 Jetson 丟的是像素，這裡可自動轉正規化並做 y 翻正。
 *
 * 使用方式（RobotContainer）：
 *  new JoystickButton(m_operatorController, ConsController.Button.BUTTON_A.id)
 *      .whileTrue(m_yolo.createVisionDriveXY(m_driveTrain));
 */
public class Yolo extends SubsystemBase {

  // ===================== 參數設定（依需求調整） =====================

  // 讀取座標模式
  private static final boolean USE_PIXELS = false;  // true: Jetson 丟像素 x,y；false: Jetson 已丟正規化 xc,yc
  private static final double IMG_W = 640.0;        // 影像寬（像素），USE_PIXELS=true 時需要
  private static final double IMG_H = 480.0;        // 影像高（像素），USE_PIXELS=true 時需要
  private static final boolean FLIP_Y = false;      // 若 Jetson 端沒有把 y 變成「上正下負」，設 true 在此翻轉

  // 訊號清理
  private static final double DEADBAND = 0.04;      // 一次性死區（避免微抖）
  private static final double CLAMP = 1.0;          // 正規化後夾限 [-CLAMP, CLAMP]

  // 映射與動態限制（給 Command 用）
  private static final double K_FWD = 1.2;          // y → 前進速度比例（m/s 每單位 y）
  private static final double K_ROT = 3.0;          // x → 角速度比例（rad/s 每單位 x）
  private static final double MAX_VX = 2.0;         // 前進速度上限 m/s
  private static final double MAX_OMEGA = Math.PI;  // 角速度上限 rad/s（約 180°/s）
  private static final double SLEW_VX = 3.0;        // 前進加速度限制 m/s^2
  private static final double SLEW_OMEGA = 6.0;     // 旋轉加速度限制 rad/s^2
  private static final double EXTRA_DEADBAND = 0.02;// 二次死區（在指令裡用）

  // ===================== 成員變數 =====================

  private final NetworkTable table;  // "coral" 表
  private double xc = Double.NaN;    // 正規化後 X（右正，-1..1）
  private double yc = Double.NaN;    // 正規化後 Y（上正，-1..1）
  private boolean hasTarget = false; // 是否有有效目標

  public Yolo() {
    table = NetworkTableInstance.getDefault().getTable("coral");
  }

  // ===================== 讀取與處理（每 20ms） =====================

  @Override
  public void periodic() {
    // 先讀數字型（優先支援 Jetson 丟出的 Number）
    double xNum = table.getEntry("xc").getDouble(Double.NaN);
    double yNum = table.getEntry("yc").getDouble(Double.NaN);

    // 若無，兼容舊鍵名字串 "x","y"
    if (Double.isNaN(xNum) || Double.isNaN(yNum)) {
      String xs = table.getEntry("x").getString("");
      String ys = table.getEntry("y").getString("");
      try {
        xNum = Double.parseDouble(xs);
        yNum = Double.parseDouble(ys);
      } catch (Exception e) {
        xNum = Double.NaN;
        yNum = Double.NaN;
      }
    }
//
    // 沒資料
    if (Double.isNaN(xNum) || Double.isNaN(yNum)) {
      hasTarget = false;
      xc = yc = Double.NaN;
      SmartDashboard.putString("coral/status", "no_target");
      pushDash();
      return;
    }

    // 轉為正規化座標
    if (USE_PIXELS) {
      // 像素 → [-1,1]，以畫面中心為原點；x 右正；y 上正
      double nx = (xNum - IMG_W / 2.0) / (IMG_W / 2.0);
      double ny = -((yNum - IMG_H / 2.0) / (IMG_H / 2.0));
      xNum = nx;
      yNum = ny;
    } else {
      // 已是 [-1,1]；若需要把 y 翻成上正下負，就翻
      if (FLIP_Y) yNum = -yNum;
    }

    // 一次性死區
    xNum = (Math.abs(xNum) < DEADBAND) ? 0.0 : xNum;
    yNum = (Math.abs(yNum) < DEADBAND) ? 0.0 : yNum;

    // 夾限
    xc = clamp(xNum, -CLAMP, CLAMP);
    yc = clamp(yNum, -CLAMP, CLAMP);
    hasTarget = true;

    SmartDashboard.putString("coral/status", "ok");
    pushDash();
  }

  // ===================== 對外查詢 =====================

  /** 是否有有效目標 */
  public boolean hasTarget() { return hasTarget; }

  /** 正規化後 X，右正左負，範圍約 [-1,1] */
  public double getXc() { return xc; }

  /** 正規化後 Y，上正下負，範圍約 [-1,1] */
  public double getYc() { return yc; }

  // ===================== 指令工廠：用 X 轉向、Y 前進 =====================

  /**
   * 建立指令：用鏡頭 X 控旋轉、用鏡頭 Y 控前進。
   * - 會在內部做二次 deadband、限幅與 Slew 限速。
   * - 無目標時輸出 0。
   */
  public Command createVisionDriveXY(DriveTrain drive) {
    // 限速器在 Command 內保持狀態
    SlewRateLimiter vxLimiter = new SlewRateLimiter(SLEW_VX);
    SlewRateLimiter omLimiter = new SlewRateLimiter(SLEW_OMEGA);

    return this.run(() -> {
      if (!hasTarget()) {
        drive.drive(new ChassisSpeeds(0, 0, 0));
        return;
      }

      double x = getXc();
      double y = getYc();

      // 二次 deadband
      x = (Math.abs(x) < EXTRA_DEADBAND) ? 0.0 : x;
      y = (Math.abs(y) < EXTRA_DEADBAND) ? 0.0 : y;

      // 比例映射
      double vxCmd = K_FWD * y;   // 前進速度 m/s
      double omCmd = K_ROT * x;   // 角速度 rad/s（x>0 逆時針）

      // 限幅
      vxCmd = clamp(vxCmd, -MAX_VX, MAX_VX);
      omCmd = clamp(omCmd, -MAX_OMEGA, MAX_OMEGA);

      // Slew 限速
      double vx = vxLimiter.calculate(vxCmd);
      double omega = omLimiter.calculate(omCmd);

      // 交給底盤。DriveTrain.drive(ChassisSpeeds) 會轉成場地座標
      drive.drive(new ChassisSpeeds(vx, 0.0, omega));
    }).finallyDo(interrupted -> {
      drive.drive(new ChassisSpeeds(0, 0, 0));
    }).withName("VisionDriveXY");
  }

  // ===================== 小工具 =====================

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  private void pushDash() {
    SmartDashboard.putNumber("coral/xc", xc);
    SmartDashboard.putNumber("coral/yc", yc);
  }
}
