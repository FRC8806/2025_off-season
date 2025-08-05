package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift;

public class putL1 extends Command {
  
  private Lift m_lift;
  private ConsLift.Pose pose;
  private boolean out = false;
 private boolean end = false;
  public putL1(Lift m_lift, ConsLift.Pose pose) {
    this.m_lift = m_lift;
    this.pose = pose;
    addRequirements(m_lift);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_lift.setPose(pose);
    end = false;
    if(m_lift.getLiftPosition()<=-17){
     out = true;
    }else{
      out = false;
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lift.setPose(pose);
    if(out){
      m_lift.setRollingSpeed(-0.3);
    }else{
      m_lift.setRollingSpeed(0);
    }
  }

//     m_lift.setPose(pose);
//     System.out.println("[PutCoral] Y Pressed! SetPose -> " + pose.name());
//   m_lift.setPose(pose);
// }
  

  // Called every time the scheduler runs while the command is scheduled.
//   @Override
  
//   public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     end = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
