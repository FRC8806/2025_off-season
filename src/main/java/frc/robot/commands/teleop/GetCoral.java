package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.constants.ConsLift;


public class GetCoral extends Command {
  private Intake m_intake;
  private Lift m_lift;

   
  private boolean isGoingUp = false;   
  private boolean end = false;     
  private boolean isGoingDown = false;     
  private boolean start = false;       
  
  public GetCoral(Intake m_intake,Lift m_lift) {
    this.m_lift = m_lift;
    addRequirements(m_lift);  
    this.m_intake = m_intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = true;
    end = false;
    m_intake.setPosition(ConsIntake.downPosition);
    m_intake.setTransportSpeed(ConsIntake.transportSpeed);
    m_intake.setRollingSpeed(ConsIntake.rollingSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {
        double lift = m_lift.getLiftPosition();
        double arm = m_lift.getArmPosition();
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
            m_lift.setPose(ConsLift.Pose.RESET_C);
            m_lift.setRollingSpeed(0);
            m_intake.setPosition(ConsIntake.upPosition);
            m_intake.setTransportSpeed(0);
            m_intake.setRollingSpeed(0);
            if(lift>=-12){
                end = true;
            }
          }
            
        }
    
      }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
