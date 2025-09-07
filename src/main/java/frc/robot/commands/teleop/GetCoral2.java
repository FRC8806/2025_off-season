package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;

public class GetCoral2 extends Command {

  private final Lift m_lift;
  private Intake m_intake;
  // 狀態變數
  private boolean isGoingDown = false; 
  private boolean isGoingUp = false;   
  private boolean isDone = false;      

  public GetCoral2(Lift m_lift,Intake m_intake) {
    this.m_lift = m_lift;
    addRequirements(m_lift);  
    this.m_intake = m_intake;
    addRequirements(m_intake);  
    }

  @Override
  public void initialize() {
    isGoingDown = true;
    isGoingUp = false;
    isDone = false;
  }

  @Override
  public void execute() {
    double lift = m_lift.getLiftPosition();
    double arm = m_lift.getArmPosition();

    if (isGoingDown) {
      m_lift.setPose(ConsLift.Pose.DOWM_CORAL);
      m_lift.setRollingSpeed(ConsLift.coralSpeed);

      boolean armReady = Math.abs(arm - 0) <= 0.1;     
      boolean liftReady = Math.abs(lift - (-0.5)) <= 1.5; 
      if (armReady && liftReady) {
        isGoingDown = false;
        isGoingUp = true; 
      }
    }

    if (isGoingUp) {
      m_lift.setPose(ConsLift.Pose.UP_CORAL);
          m_lift.setRollingSpeed(0);// }
      if (lift <= -10) {
        isGoingUp = false;
        isDone = true; 
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_lift.setRollingSpeed(0);
    isGoingDown = false;
    isGoingUp = false;
    isDone = false;
  }
  
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
