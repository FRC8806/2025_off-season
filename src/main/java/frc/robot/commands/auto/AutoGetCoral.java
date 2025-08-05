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

  private boolean speedOK = false;
  private boolean transportTripped = false;
  private boolean liftStarted = false;
  private boolean isDone = false;

  private final Timer timer = new Timer();

  public AutoGetCoral(Intake intake, Lift lift) {
    this.m_intake = intake;
    this.m_lift = lift;
    addRequirements(intake, lift);
  }

  @Override
  public void initialize() {
    speedOK = false;
    transportTripped = false;
    liftStarted = false;
    isDone = false;

    // 啟動 Intake 吸 Coral
    m_intake.setPosition(ConsIntake.downPosition);
    m_intake.setTransportSpeed(ConsIntake.transportSpeed);
    m_intake.setRollingSpeed(ConsIntake.rollingSpeed);
  }

  @Override
  public void execute() {
    double speed = m_intake.getTransportSpeed();

    // 第一階段：檢查是否碰到 Coral（速度高）
    if (speed > 30) {
      speedOK = true;
    }

    // 第二階段：Coral 被吸進去 → 速度變低，啟動 lift 動作
    if (speedOK && speed < 29.6 && !transportTripped) {
      m_intake.setPosition(ConsIntake.upPosition);
      m_intake.setRollingSpeed(0);
      timer.reset();
      timer.start();
      transportTripped = true;
    }

    // 第三階段：等 0.3 秒後再執行 lift
    if (transportTripped && !liftStarted && timer.hasElapsed(0.3)) {
      m_intake.setTransportSpeed(0);
      m_lift.setPose(ConsLift.Pose.DOWM_CORAL);
      m_lift.setRollingSpeed(ConsLift.coralSpeed);
      liftStarted = true;
      timer.stop();
    }

    // 第四階段：lift 到達指定位置後完成命令
    if (liftStarted && m_lift.getLiftPosition() <= ConsLift.Pose.DOWM_CORAL.pos_lift + 0.1) {
      isDone = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // 可依需求關閉動作
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
