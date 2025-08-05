package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;

public class TrackingCoral extends Command {

  double x, y;


  public TrackingCoral(double x, double y) {
    this.x = x;
    this.y = y;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    pose.x + x, pose.y + y (加減不確定)
    pose.xy要從中心點往鏡頭那邊加減
    可以配校正角度?
     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
