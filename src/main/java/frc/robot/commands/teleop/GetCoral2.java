package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;

public class GetCoral2 extends Command {
  private Lift m_lift;

  private boolean isGoingDown = false;
  private boolean isGoingUp = false;
  private boolean isDone = false;

  public GetCoral2(Lift m_lift) {
    this.m_lift = m_lift;
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isGoingDown = true;
    isGoingUp = false;
    isDone = false;
  }
    


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_lift.getCoral();
   if(isGoingDown){
     m_lift.setPose(ConsLift.Pose.DOWM_CORAL);
     m_lift.setRollingSpeed(ConsLift.coralSpeed); 
     isGoingDown = false;
     isDone = true;
    //  isGoingUp = true;
   }

    if (isGoingUp && m_lift.getLiftPosition()>=-1.2) {
       m_lift.setPose(ConsLift.Pose.UP_CORAL);
        m_lift.setRollingSpeed(0); 
        isDone = true;
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     isGoingDown = false;
    isGoingUp = false;
    isDone = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone = true;
  }
}