package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Lift;

public class L1 extends Command {
  private Lift m_lift;
  private ConsLift.Pose pose;
  private boolean finish = false, isdown = false, isup = false, output = false;

  /** Creates a new PutAlgae. */
  public L1(Lift m_lift, ConsLift.Pose pose) {
    this.m_lift = m_lift;
    this.pose = pose;
    addRequirements(m_lift);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double arm = m_lift.getArmPosition();
    double lift = m_lift.getLiftPosition();
    if (Math.abs(arm - (-0.15)) <= 0.02 && Math.abs(lift - (-18)) <= 2) {
      isdown = false;
      isup = true;
    } else {
      isdown = true;
      isup = false;
    }
  }

  @Override
  public void execute() {
    // m_lift.setPose(pose);

    if (isup) {
      m_lift.setPose(pose);
      m_lift.setRollingSpeed(-0.25);
    }
    if (isdown) {
      m_lift.setPose(pose);
      m_lift.setRollingSpeed(0);
      // isdown = false;
    }
    SmartDashboard.putBoolean("isdown", isdown);
    SmartDashboard.putBoolean("isup", isup);

  }

  @Override
  public void end(boolean interrupted) {
    isdown = false;
    isup = false;
  }

  @Override
  public boolean isFinished() {
    return finish;
  }
}
