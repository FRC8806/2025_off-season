package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsIntake;
import frc.robot.subsystems.Intake;

public class Outputcoral extends Command {
  private final Intake m_intake;

  public Outputcoral(Intake intake) {
    this.m_intake = intake;
    addRequirements(m_intake);
    
  }

  // 按下時觸發一次，可用於初始化狀態
  @Override
  public void initialize() {
  }

  // 按住期間每個 scheduler 週期都會呼叫
  @Override
  public void execute() {
    if (m_intake.getPosition() > -18.5) {
      m_intake.setRollingSpeed(0);
      m_intake.setTransportSpeed(0);
    } else {
      m_intake.setRollingSpeed(-0.3);
      m_intake.setTransportSpeed(-0.3);
    }
  }

  // 當按鈕放開（指令被中斷）時會呼叫，確保馬達停轉
  @Override
  public void end(boolean interrupted) {
    m_intake.setRollingSpeed(ConsIntake.rollingSpeed);
    m_intake.setTransportSpeed(ConsIntake.transportSpeed);
    m_intake.setPosition(ConsIntake.downPosition);

  }

  // 始終為 false，讓指令持續執行直到被外部中斷
  @Override
  public boolean isFinished() {
    return false;
  }
}
