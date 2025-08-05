package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;

public class AllDefaultCommand extends Command {

  private Intake m_intake;
  private Lift m_lift;
  private Supplier<Double> intakeRolling;
  private Supplier<Double> intakeTransport;
  private Supplier<Boolean> intakeUp;
  private Supplier<Boolean> intakeDown;
  private Supplier<Double> liftControl;
  private Supplier<Double> armControl;
  private Supplier<Boolean> getCoral;



  public AllDefaultCommand(Intake m_intake, Lift m_lift, 
    Supplier<Double> intakeRolling, Supplier<Double> intakeTransport, Supplier<Boolean> intakeUp, Supplier<Boolean> intakeDown, 
    Supplier<Double> liftControl, Supplier<Double> armControl, Supplier<Boolean> getCoral) {
    this.m_intake = m_intake;
    this.m_lift = m_lift;
    this.intakeRolling = intakeRolling;
    this.intakeTransport = intakeTransport;
    this.intakeUp = intakeUp;
    this.intakeDown = intakeDown;
    this.liftControl = liftControl;
    this.armControl = armControl;
    this.getCoral = getCoral;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setAngleSpeed(intakeUp.get() ? 0.3 : intakeDown.get() ? -0.3 : 0);//上下相反的話自己改
    m_intake.setRollingSpeedout(intakeRolling.get());//方向自己確認 自行改數字測試 測完到constants裡面紀錄
    m_intake.setTransportSpeed(intakeTransport.get());//同上

    m_lift.setArmSpeed(armControl.get() * 0.4);//自己調整
    m_lift.setLiftSpeed(liftControl.get() * 0.4);
    m_lift.setRollingSpeed(getCoral.get() ? 0.3 : 0);//正反不知道對不對 自己調整方向速度
    m_lift.setRollingSpeed(getCoral.get() ? 0.3 : 0);
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
