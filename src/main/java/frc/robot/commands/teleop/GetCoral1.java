package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsIntake;
import frc.robot.subsystems.Intake;

public class GetCoral1 extends Command {
  private Intake m_intake;

  public GetCoral1(Intake m_intake) {
    this.m_intake = m_intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setPosition(ConsIntake.downPosition);
    m_intake.setTransportSpeed(ConsIntake.transportSpeed);
    m_intake.setRollingSpeed(ConsIntake.rollingSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setPosition(ConsIntake.upPosition);
    m_intake.setTransportSpeed(0);
    m_intake.setRollingSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
