package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.ConsClimber;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Climber;

public class Climberup extends Command {

  private final Lift m_lift;
  private final Climber m_climber1;
  private final ConsLift.Pose pose;
  private boolean wantup = false, wantupup = false, end = false;

  public Climberup(Lift m_lift, Climber m_climber, ConsLift.Pose pose) {
    this.m_lift = m_lift;
    this.m_climber1 = m_climber;
    this.pose = pose;
    addRequirements(m_lift, m_climber);
  }

  @Override
  public void initialize() {
    m_lift.setRollingSpeed(0);

  }

  @Override
  public void execute() {

    m_climber1.setPosition(ConsClimber.readyPosition);
    m_lift.setPose(pose);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return end;
  }
}