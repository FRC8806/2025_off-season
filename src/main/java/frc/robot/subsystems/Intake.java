package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConsIntake;

public class Intake extends SubsystemBase {

  private TalonFX leftTransportMotor = new TalonFX(ConsIntake.LEFT_TRANSPORT_MOTOR_ID);
  private TalonFX rightTransportMotor = new TalonFX(ConsIntake.RIGHT_TRANSPORT_MOTOR_ID);
  private TalonFX angleMotor = new TalonFX(ConsIntake.ANGLE_MOTOR_ID);
  private TalonFX rollingMotor = new TalonFX(ConsIntake.ROLLING_MOTOR_ID);

  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);


//   private boolean isUp = false;


  public Intake() {
    //angle motor
    angleMotor.getConfigurator().apply(ConsIntake.angleConfigs);
    angleMotor.setNeutralMode(NeutralModeValue.Brake);
  
    //transport
    leftTransportMotor.setNeutralMode(NeutralModeValue.Coast);
    rightTransportMotor.setNeutralMode(NeutralModeValue.Coast);

    //rolling
    rollingMotor.setNeutralMode(NeutralModeValue.Coast);
  
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake position", getPosition());
    SmartDashboard.putNumber("transport speed", getTransportSpeed());

    
  }
  public void setRollingSpeedout(double speed){
    rollingMotor.set(-speed);
  }
  public void setRollingSpeed(double speed){
    rollingMotor.set(speed);
  }

  public void setTransportSpeed(double speed){
    rightTransportMotor.set(-speed);
    leftTransportMotor.set(speed);//正反自己測
  }

  public void getCoral(){
    setRollingSpeed(ConsIntake.rollingSpeed);
    setTransportSpeed(ConsIntake.transportSpeed);
  }

  public void stop(){
    setRollingSpeed(0);
    setTransportSpeed(0);
  }

  public void setAngleSpeed(double speed){
    angleMotor.set(speed);
  }

  public double getPosition(){
    return angleMotor.getPosition().getValueAsDouble();
  }
  public double getrollingspeed(){
    return rollingMotor.getVelocity().getValueAsDouble();
  }

  public double getTransportSpeed(){
    return leftTransportMotor.getVelocity().getValueAsDouble();
  }

  public void setPosition(double position){
    angleMotor.setControl(m_request.withPosition(position));
  }

}
