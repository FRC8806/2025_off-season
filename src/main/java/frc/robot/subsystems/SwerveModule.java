package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ConsSwerve;

public class SwerveModule { 
  private TalonFX m_throttle;
  public TalonFX m_rotor;
  public CANcoder m_encoder;
  private VelocityVoltage m_velocitySetter;
  private PositionVoltage m_angleSetter;
  public double angle;

  public SwerveModule(ConsSwerve.Modules module) {
    m_throttle = new TalonFX(module.throttle_id);
    m_rotor = new TalonFX(module.rotor_id);
    m_encoder = new CANcoder(module.encoder_id);
    m_velocitySetter = new VelocityVoltage(0);
    m_angleSetter = new PositionVoltage(0);
    m_velocitySetter.Slot = 0;
    m_angleSetter.Slot = 0;
    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    m_throttle.getConfigurator().apply(talonConfigs);
    m_throttle.getConfigurator().apply(ConsSwerve.driveGains);

    talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();
    // talonConfigs.Slot0 = ConsSwerve.steerGains;
    talonConfigs.Feedback.FeedbackRemoteSensorID = module.encoder_id;
    talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    talonConfigs.Feedback.RotorToSensorRatio = 150/7;
    talonConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    talonConfigs.Slot0 = ConsSwerve.rotorGains;
    talonConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_rotor.getConfigurator().apply(talonConfigs);

    m_throttle.setNeutralMode(NeutralModeValue.Brake);
    m_rotor.setNeutralMode(NeutralModeValue.Coast);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = module.offset;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    m_encoder.getConfigurator().apply(config);
  }

  public void setState(SwerveModuleState state) {
    state.optimize(getAngle());
    double throttleOutput = state.speedMetersPerSecond / ConsSwerve.kThrottleConversionFactor;
    double rotorOutput = state.angle.getRotations();
    m_throttle.setControl(m_velocitySetter.withVelocity(throttleOutput));
    m_rotor.setControl(m_angleSetter.withPosition(rotorOutput));
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(m_encoder.getPosition().getValueAsDouble());
  }

  public double getCancoder() {
    // return Rotation2d.fromRotations(m_encoder.getAbsolutePosition().getValueAsDouble());
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_throttle.getVelocity().getValueAsDouble() * ConsSwerve.kThrottleConversionFactor,
        getAngle());
  }

  public SwerveModulePosition getPosition() {
		double throttlePosition = m_throttle.getPosition().getValueAsDouble() * ConsSwerve.kThrottleConversionFactor;
		return new SwerveModulePosition(throttlePosition,
				Rotation2d.fromRotations(m_encoder.getAbsolutePosition().getValueAsDouble()));
	}
}
