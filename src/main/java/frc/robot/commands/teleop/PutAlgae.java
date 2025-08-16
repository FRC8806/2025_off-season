package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConsLift;
import frc.robot.subsystems.Lift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PutAlgae extends Command {
  private Lift m_lift;
  private ConsLift.Pose pose;
  private boolean finish = false, isdown = false, isup = false, output = false;

  /** Creates a new PutAlgae. */
  public PutAlgae(Lift m_lift, ConsLift.Pose pose) {
    this.m_lift = m_lift;
    this.pose = pose;
    addRequirements(m_lift);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double arm = m_lift.getArmPosition();
    double lift = m_lift.getLiftPosition();
    if (arm <= -0.38 && lift <= -57) {
      isdown = false;
      isup = true;
    } else {
      isdown = true;
      isup = false;
    }

    // 黑洞
    if (arm > -0.15 && lift >= -15) {
      isdown = false;
      isup = true;
    } else {
      isdown = true;
      isup = false;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_lift.setPose(pose);

    if (isup) {
      m_lift.setPose(pose);
      m_lift.setRollingSpeed(-0.9);
    }
    if (isdown) {
      m_lift.setPose(pose);
      m_lift.setRollingSpeed(0.25);
      // isdown = false;
    }
    SmartDashboard.putBoolean("isdown", isdown);
    SmartDashboard.putBoolean("isup", isup);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isdown = false;
    isup = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
