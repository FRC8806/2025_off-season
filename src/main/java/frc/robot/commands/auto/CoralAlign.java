
package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Yolo;

public class CoralAlign extends Command {
  private Yolo yolo;
  private DriveTrain driveTrain;
  private Boolean isTracking = false;

  private double targetX = 0, targetY = 0.2;
  private double xC = 0.0, yC = 0.0;
  private double lastxC = 0.0, lastyC = 0.0;

  private PIDController vxController = new PIDController(7, 0, 0);
  private PIDController vzController = new PIDController(6, 0.6, 0);
  private int lostTime = 0;
  private Timer detectedTime = new Timer();

  public CoralAlign(Yolo yolo, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    this.yolo = yolo;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    lostTime = 0;
    detectedTime.reset();
  }

  @Override
  public void execute() {
    if (yolo.hasTarget()) {
      detectedTime.start();
    }
    if (detectedTime.get() > 0.2) {
      isTracking = true;
    }
    if (isTracking && isSameCoral() == false) {
      lostTime++;
    }

    if (isTracking && yolo.hasTarget() == false) {
      lostTime ++;
    }
    SmartDashboard.putBoolean("isTracking", isTracking);
    SmartDashboard.putBoolean("notSameCoral", isSameCoral() == false);
    SmartDashboard.putBoolean("nonTarget", yolo.hasTarget() == false);
    SmartDashboard.putNumber("lostTime", lostTime);

    // if () {
    //   driveTrain.autoDrive(new ChassisSpeeds(0.5, 0, 0));
    //   return;
    // }
    yC = yolo.getYc();
    xC = yolo.getXc();

    if (isTracking && isSameCoral()) {
      double eX = targetX - xC;
      double eY = targetY - yC;
      double vz = vzController.calculate(-eX);
      double vx = vxController.calculate(-eY);
      driveTrain.autoDrive(new ChassisSpeeds(vx, 0, vz));
    }
    lastxC = xC;
    lastyC = yC;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return isTracking && lostTime > 20;
  }

  private boolean isSameCoral() {
    if (Math.abs(xC - lastxC) <= 0.01 && Math.abs(yC - lastyC) <= 0.01) {
      return true;
    } else {
      return false;
    }
  }
}
