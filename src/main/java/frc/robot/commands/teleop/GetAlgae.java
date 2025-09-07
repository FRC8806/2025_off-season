package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Lift;

public class GetAlgae extends Command {
    private Lift m_lift;
  private ConsLift.Pose pose;
  private boolean end = false;     

  /** Creates a new PutAlgae. */
  public GetAlgae(Lift m_lift, ConsLift.Pose pose) {
    this.m_lift = m_lift;
    this.pose = pose;
    addRequirements(m_lift);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.setRollingSpeed(0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lift.setRollingSpeed(0.3);
    m_lift.setPose(pose);
    if(m_lift.isFinished(pose)){
      end = true;
  }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_lift.setRollingSpeed(0.3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_lift.isFinished(pose) || end ;
    
  } 
}
