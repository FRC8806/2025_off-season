package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain;

public class ZeroSpeed extends Command {
  private final DriveTrain m_driveTrain;

  public ZeroSpeed(DriveTrain driveTrain) {
    this.m_driveTrain = driveTrain;
    addRequirements(driveTrain); // 下一個需要 DriveTrain 的 command 會自動中斷它
  }

  @Override
  public void initialize() {
    applyXLock();
  }

  @Override
  public void execute() {
    applyXLock(); // 2025 WPILib 建議持續輸出，避免角度飄
  }

  private void applyXLock() {
    SwerveModuleState[] lockStates = {
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),   // FL
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),  // FR
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),  // BL
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))    // BR
    };
    m_driveTrain.setModuleStates(lockStates);
  }

  @Override
  public boolean isFinished() {
    return false; // 保持到被其他 command 打斷
  }
}
