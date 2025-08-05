package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift;

public class L1 extends Command {
  
  private Lift m_lift;
  private ConsLift.Pose pose;
    private boolean wantout = false,finish=false;
 private boolean test = false;
  public L1(Lift m_lift) {
    this.m_lift = m_lift;
    this.pose = pose;
    addRequirements(m_lift);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_lift.setPose(pose);
    if(m_lift.getLiftPosition()<=-13){
    wantout = true;
    }else{
     m_lift.setRollingSpeed(0);
     finish = true;
    }
    // test = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(wantout && m_lift.getLiftPosition()<=-16){
    m_lift.setRollingSpeed(-0.3);

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
    finish = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish = true;
  }
}