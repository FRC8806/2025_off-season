package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.ConsClimber;
import frc.robot.subsystems.Climber;

public class ClimberDefaultCommand extends Command {
  /** Creates a new ClimberDefaultCommand. */
  private Climber m_climber;
  private Supplier<Boolean> up;
  private Supplier<Boolean> down;

  public ClimberDefaultCommand(Climber m_climber, Supplier<Boolean> up, Supplier<Boolean> down) {
    this.m_climber = m_climber;
    this.up = up;
    this.down = down;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      //m_climber.setPosition(ConsClimber.readyPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setSpeed(up.get() ? 0.9 : down.get() ? -0.9 : 0);//測完可以直接改過去 robotContainer要改成按鈕
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_climber.setPosition(ConsClimber.finalPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
