package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj.Timer;

public class GetCoral1 extends Command {

  private Intake m_intake;
  private Lift m_lift;
  private boolean armGet = false, intakeGet = false;
  private boolean speedOK = false,StopTransport  = false,isup = false;
  private boolean finished = false;

  private final Timer timer = new Timer();

//   private boolean speedOK = false;
//   private boolean isUp = false;

  public GetCoral1(Intake m_intake, Lift m_lift) {
    this.m_intake = m_intake;
    this.m_lift = m_lift;
    addRequirements(m_lift);
    addRequirements(m_intake);
  }   

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_intake.getPosition()>-1){
      intakeGet = false;
      speedOK = false;
      StopTransport = false;
      
      m_intake.setPosition(ConsIntake.downPosition);
      m_intake.setTransportSpeed(ConsIntake.transportSpeed);
      m_intake.setRollingSpeed(ConsIntake.rollingSpeed);  
     }else{
      intakeGet = false;
      speedOK = false;
      m_intake.setPosition(ConsIntake.upPosition);
      m_intake.setTransportSpeed(0);
     m_intake.setRollingSpeed(0);
     }
    // speedOK = false;
    // m_intake.setPosition(ConsIntake.downPosition);
    // // m_intake.setPositionManual(ConsIntake.downPosition);
    // m_intake.setTransportSpeed(ConsIntake.transportSpeed);
    // m_intake.setRollingSpeed(ConsIntake.rollingSpeed);

//     speedOK = false;
//     m_intake.setPosition(ConsIntake.downPosition);
//     m_intake.setTransportSpeed(ConsIntake.transportSpeed);
//     m_intake.setRollingSpeed(ConsIntake.rollingSpeed);
    // if (!isUp) {
    //   m_intake.setPosition(ConsIntake.upPosition);
    //   isUp = true;
    //   m_intake.setRollingSpeed(0);
    //   m_intake.setTransportSpeed(0);  

    // } else {
    //   m_intake.setPosition(ConsIntake.downPosition);
    //   isUp = false;
    //   m_intake.setTransportSpeed(ConsIntake.transportSpeed);
    //   m_intake.setRollingSpeed(ConsIntake.rollingSpeed);
    // }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intakeGet = false;
   if(m_intake.getTransportSpeed()>30){
    speedOK = true;
   }
    //m_intake.getCoral();
    if (speedOK && m_intake.getTransportSpeed() < 29.6 && !StopTransport) {
      // 條件剛達成，啟動延遲倒數
      m_intake.setPosition(ConsIntake.upPosition); // 執行一次
      m_intake.setRollingSpeed(0);
      m_lift.setArmSpeed(0);
      m_lift.setLiftSpeed(0);
      m_lift.setRollingSpeed(0); 
  
      timer.reset();
      timer.start();
      StopTransport = true;
    }
    if (StopTransport && timer.hasElapsed(0.3)) {
      m_intake.setTransportSpeed(0);
      timer.stop();

//    if(m_intake.getTransportSpeed()>100){
//     speedOK = true;
//    }
    //m_intake.getCoral();
//     if(speedOK && m_intake.getTransportSpeed()<50) { //3000 隨便寫的 測了才知道 然後看要不要加delay // 7/4後加m_intake.getTransportSpeed()>1500 &&
//       m_intake.setPosition(ConsIntake.upPosition);
//       intakeGet = true;
//       speedOK = false;
      // m_intake.stop();
      //轉動測完角度那顆再補回來

    }
      //m_intake.stop(); //轉動測完角度那顆再補回來
    
    // if(intakeGet){
    //     armGet = m_lift.getCoral();
    // }
    SmartDashboard.putNumber("transport speed", m_intake.getTransportSpeed());
    SmartDashboard.putBoolean("speedOK", speedOK);
    SmartDashboard.putBoolean("intakeGet", intakeGet);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intakeGet = false;
    speedOK = false;
    
//     m_intake.setPosition(ConsIntake.upPosition);
//     m_intake.setRollingSpeed(0);
//     m_intake.setTransportSpeed(0);
//     m_lift.setArmSpeed(0);
//     m_lift.setLiftSpeed(0);
//     m_lift.setRollingSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return intakeGet;
    // return Math.abs(m_intake.getPosition()-ConsIntake.upPosition) < 0.3;
  }
}

