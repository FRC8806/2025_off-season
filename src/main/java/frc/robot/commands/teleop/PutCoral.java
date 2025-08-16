package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Lift;

public class PutCoral extends Command {
  private Lift m_lift;
  private ConsLift.Pose pose;

  public PutCoral(Lift m_lift, ConsLift.Pose pose) {
    this.m_lift = m_lift;
    this.pose = pose;
    addRequirements(m_lift);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.setRollingSpeed(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lift.setPose(pose);
    SmartDashboard.putBoolean("test", m_lift.isFinished(pose));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_lift.isFinished(pose);
  }
}
