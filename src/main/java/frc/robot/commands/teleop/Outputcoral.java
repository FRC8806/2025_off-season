package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj.Timer;
public class Outputcoral extends Command {

  private Intake m_intake;
  private boolean  intakeGet = false;

  private final Timer timer = new Timer();

//   private boolean speedOK = false;
//   private boolean isUp = false;

  public Outputcoral(Intake m_intake) {
    this.m_intake = m_intake;
    addRequirements(m_intake);
  }   

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_intake.getPosition()>-18.5){
      m_intake.setRollingSpeed(0);  
     }else{
      m_intake.setRollingSpeed(-0.3); 
      m_intake.setTransportSpeed(-0.3); 
     }
     }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return intakeGet;
  }
}


