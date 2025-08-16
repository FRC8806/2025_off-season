package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Lift;

public class AutoPutCoral extends Command {
  private Lift m_lift;
  private ConsLift.Pose pose;

  public AutoPutCoral(Lift m_lift, ConsLift.Pose pose) {
    this.m_lift = m_lift;
    this.pose = pose;
    addRequirements(m_lift);
  }

  @Override
  public void initialize() {
    m_lift.setRollingSpeed(0);
    m_lift.set(pose); // 改用 set() 強制推進機構
  }

  @Override
  public void execute() {
    m_lift.set(pose);  // 連續呼叫以確保更新
  }

  @Override
  public boolean isFinished() {
    boolean finish = m_lift.isFinished(pose);
    SmartDashboard.putBoolean("AutoPutCoral_Finished", finish);
    return finish;
  }

  @Override
  public void end(boolean interrupted) {
    m_lift.setRollingSpeed(0);
  }
}
