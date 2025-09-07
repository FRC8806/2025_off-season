package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ConsClimber;

public class Climber extends SubsystemBase {
  private TalonFX climbMotor = new TalonFX(ConsClimber.CLIMB_MOTOR_ID);
  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  public Climber() {
    climbMotor.getConfigurator().apply(ConsClimber.climberConfigs);
    climbMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("climber angle", getPosition());
  }

  public void setSpeed(double speed) {
    climbMotor.set(speed);
  }

  public double getPosition() {
    return climbMotor.getPosition().getValueAsDouble();
  }

  public void setPosition(double position){
    climbMotor.setControl(m_request.withPosition(position));
  }

  
}
