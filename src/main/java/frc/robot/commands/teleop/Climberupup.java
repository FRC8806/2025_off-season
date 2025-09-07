package frc.robot.commands.teleop;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.ConsClimber;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Climber;

public class Climberupup extends Command {

  private final Lift m_lift;
  private final Climber m_climber;
  private final ConsLift.Pose pose;
  private boolean wantup = false, wantupup = false, end = false, test;

  public Climberupup(Lift m_lift, Climber m_climber, ConsLift.Pose pose) {
    this.m_lift = m_lift;
    this.m_climber = m_climber;
    this.pose = pose;
    addRequirements(m_lift, m_climber);
  }

  @Override
  public void initialize() {
    m_lift.setRollingSpeed(0);
    // if(m_climber.getPosition()>=-20){
    // wantup = true ;
    // }else if(m_climber.getPosition()<=-110){
    // wantupup = true ;
    // }
  }

  @Override
  public void execute() {
    // if (wantup) {
    // m_climber.setPosition(ConsClimber.readyPosition);
    // m_lift.setPose(pose);
    // wantup = false;
    // if(m_climber.getPosition()<=-110){
    // end = true;
    // }
    // }else if(wantupup){
    m_climber.setPosition(ConsClimber.finalPosition);
    m_lift.setPose(pose);
    // wantupup = false;
    // if(m_climber.getPosition()<=-180){
    // end = true;
    // }
    // }
    // SmartDashboard.putBoolean("wantup", wantup);
    // SmartDashboard.putBoolean("wantupup", wantupup);
    // SmartDashboard.putBoolean("end", end);

  }

  @Override
  public void end(boolean interrupted) {
    end = false;

  }

  @Override
  public boolean isFinished() {
    return test;
  }
}
