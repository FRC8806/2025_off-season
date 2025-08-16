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

    // 啟動 Intake 吸 Coral
    m_intake.setPosition(ConsIntake.downPosition);
    m_intake.setTransportSpeed(ConsIntake.transportSpeed);
    m_intake.setRollingSpeed(ConsIntake.rollingSpeed);
  }

  @Override
  public void execute() {
    double Angle = m_intake.getPosition();

    if (Angle >= -2) {
      isDone = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setTransportSpeed(ConsIntake.transportSpeed);
    m_intake.setRollingSpeed(ConsIntake.rollingSpeed);

  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
