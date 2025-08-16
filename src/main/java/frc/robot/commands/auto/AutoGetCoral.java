package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsIntake;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj.Timer;

public class AutoGetCoral extends Command {
  private final Intake m_intake;
  private final Lift m_lift;
  
  private boolean isGoingUp = false;   
  private boolean isDone = false;     
  private boolean isGoingDown = false;     
  private boolean start = false;     

  private final Timer timer = new Timer();

  public AutoGetCoral(Intake intake, Lift lift) {
    this.m_intake = intake;
    this.m_lift = lift;
    addRequirements(intake, lift);
  }

  @Override
  public void initialize() {
    
    isGoingUp = false;  
    isDone = false;     
    isGoingDown = false;
    start = true;     
    m_intake.setPosition(ConsIntake.downPosition);
    m_intake.setTransportSpeed(ConsIntake.transportSpeed);
    m_intake.setRollingSpeed(ConsIntake.rollingSpeed);
  }

  @Override
  public void execute() {
    double lift = m_lift.getLiftPosition();
    double arm = m_lift.getArmPosition();
  //  boolean isGoingDown = m_intake.getIR()>= 25;
    if(start && m_intake.getIR()>= 25){
      isGoingDown = true;
      start = false;
    }
    if(isGoingDown){
      m_lift.setPose(ConsLift.Pose.DOWM_CORAL);
      m_lift.setRollingSpeed(ConsLift.coralSpeed);

      boolean armReady = Math.abs(arm - 0) <= 0.03;     
      boolean liftReady = Math.abs(lift - (0.2)) <= 1; 
      if (armReady && liftReady) {
        isGoingDown = false;
        isGoingUp = true; 
      }
    }
  
    if (isGoingUp) {
        m_lift.setPose(ConsLift.Pose.UP_CORAL);
          m_lift.setRollingSpeed(0);// }
          m_intake.setPosition(ConsIntake.upPosition);
          m_intake.setTransportSpeed(0);
          m_intake.setRollingSpeed(0);

      if (lift <= -10) {
        isGoingUp = false;
        isDone = true; 
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
     isGoingUp = false;
     isDone = false;
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
