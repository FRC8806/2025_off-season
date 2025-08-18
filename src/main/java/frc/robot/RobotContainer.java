package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.*;
import frc.robot.commands.teleop.*;
import frc.robot.constants.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final Intake m_intake = new Intake();
  private final DriveTrain m_driveTrain;
  private final Vision m_vision;
  private final Lift m_lift = new Lift();
  private final Climber m_climber = new Climber();
  private final Yolo m_yolo = new Yolo(); 

  private final XboxController m_driveController = new XboxController(ConsController.kDriveControllerPort);
  private final XboxController m_operatorController = new XboxController(ConsController.kOperatorControllerPort);
  private final XboxController m_buttonBroadController = new XboxController(ConsController.kButtonBroadControllerPort);

  public Supplier<Boolean> isRedAliance;
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer(Supplier<Boolean> isRedAliance) {
    m_driveTrain = new DriveTrain(isRedAliance);
    m_vision = new Vision(m_driveTrain);
    this.isRedAliance = isRedAliance;
    configureBindings();
    setDefaultCommand();
    namedCommand();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void setDefaultCommand() {
    m_climber.setDefaultCommand(new ClimberDefaultCommand(m_climber,
    () -> m_operatorController.getRightBumperButton(),
    () -> m_operatorController.getLeftBumperButton()));

    // 1) 底盤預設：搖桿駕駛（沒有按鈕時就用這個）
    m_driveTrain.setDefaultCommand(
        new RunCommand(
            () -> m_driveTrain.drive(
                -getDriveControllerAxisOnDeadBand(m_driveController.getLeftY(), 2) * ConsSwerve.throttleMaxSpeed,
                -getDriveControllerAxisOnDeadBand(m_driveController.getLeftX(), 2) * ConsSwerve.throttleMaxSpeed,
                getDriveControllerAxisOnDeadBand(m_driveController.getRightX(), 2) * ConsSwerve.kMaxRotationSpeed),
            m_driveTrain));
  }

  private void configureBindings() {
    new Trigger(() -> m_driveController.getXButton()).toggleOnTrue(new CoralAlign(m_yolo, m_driveTrain));
    // 3) 按住 → 跑 AprilTag2；放開 → 自動 end()（你會在 end() 裡 cancel + stop）
    new Trigger(
        () -> (m_buttonBroadController.getYButton() ||
            m_buttonBroadController.getLeftBumperButton() ||
            m_buttonBroadController.getRightBumperButton() ||
            m_buttonBroadController.getLeftTriggerAxis() == 1 ||
            m_buttonBroadController.getRightTriggerAxis() == 1 ||
            m_buttonBroadController.getLeftX() == -1 ||
            m_buttonBroadController.getLeftX() == 1 ||
            m_buttonBroadController.getLeftY() == 1 ||
            m_buttonBroadController.getLeftY() == -1 ||
            m_buttonBroadController.getAButton() ||
            m_buttonBroadController.getBButton() ||
            m_buttonBroadController.getXButton()))
        .whileTrue(
            new ReefAlign(
                m_driveTrain,
                () -> m_buttonBroadController.getYButton(),
                () -> m_buttonBroadController.getLeftBumperButton(),
                () -> m_buttonBroadController.getRightBumperButton(),
                () -> m_buttonBroadController.getLeftTriggerAxis(),
                () -> m_buttonBroadController.getRightTriggerAxis(),
                () -> m_buttonBroadController.getLeftX(),
                () -> m_buttonBroadController.getLeftX(),
                () -> m_buttonBroadController.getLeftY(),
                () -> m_buttonBroadController.getLeftY(),
                () -> m_buttonBroadController.getAButton(),
                () -> m_buttonBroadController.getBButton(),
                () -> m_buttonBroadController.getXButton(),
                () -> m_driveController.getLeftX(),
                () -> m_driveController.getLeftY(),
                () -> m_driveController.getRightX(),
                isRedAliance));
    // new JoystickButton(m_operatorController,
    //     ConsController.Button.BUTTON_A.id).toggleOnTrue(
    //         new PutCoral(m_lift,
    //             ConsLift.Pose.RESET_C));
    // new JoystickButton(m_operatorController,
    //     ConsController.Button.BUTTON_B.id).toggleOnTrue(
    //         new PutCoral(m_lift,
    //             ConsLift.Pose.L3));
    // new JoystickButton(m_operatorController,
    //     ConsController.Button.BUTTON_Y.id).toggleOnTrue(
    //         new PutCoral(m_lift,
    //             ConsLift.Pose.L4));
    // new JoystickButton(m_operatorController,
    //     ConsController.Button.BUTTON_X.id).toggleOnTrue(
    //         new PutCoral(m_lift,
    //             ConsLift.Pose.L2));
    // new JoystickButton(m_operatorController,
    //     ConsController.Button.BUTTON_C.id).toggleOnTrue(
    //         new GetCoral2(m_lift,
    //             m_intake));
    // new JoystickButton(m_operatorController, ConsController.Button.BUTTON_Z.id)
    //     .onTrue(new L1(m_lift, ConsLift.Pose.L1));
    // new JoystickButton(m_operatorController,
    //     ConsController.Button.BUTTON_Z.id).onFalse(new L1(m_lift, ConsLift.Pose.L1));

    // /*
    //  * =============================================================================
    //  * =====================================================================
    //  */
    // new JoystickButton(m_driveController,
    //     ConsController.Button.BUTTON_START.id).onTrue(
    //         new Climberupup(m_lift,
    //             m_climber, ConsLift.Pose.climber));
    // new JoystickButton(m_driveController,
    //     ConsController.Button.BUTTON_BACK.id).onTrue(
    //         new Climberup(m_lift, m_climber,
    //             ConsLift.Pose.climber));
    new JoystickButton(m_driveController,
        ConsController.Button.BUTTON_RB.id).whileTrue(new GetCoral1(m_intake));
    new JoystickButton(m_driveController,
        ConsController.Button.BUTTON_LB.id).whileTrue(new Outputcoral(m_intake));
    // /*
    //  * =============================================================================
    new JoystickButton(m_driveController, ConsController.Button.BUTTON_B.id).toggleOnTrue(new GetCoral(m_intake,m_lift));
    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_X.id).toggleOnTrue(new RunToCoral(m_driveTrain,m_yolo));
    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_X.id).whileTrue(new RunToCoral(m_driveTrain, m_yolo));
    //  * =======================================================================
    //  */
    // new JoystickButton(m_driveController,
    //     ConsController.Button.BUTTON_B.id).toggleOnTrue(
    //         new GetAlgae(m_lift,
    //             ConsLift.Pose.L3A));
    // new JoystickButton(m_driveController,
    //     ConsController.Button.BUTTON_X.id).toggleOnTrue(
    //         new GetAlgae(m_lift,
    //             ConsLift.Pose.L2A));
    // new JoystickButton(m_driveController,
    //     ConsController.Button.BUTTON_Y.id).onTrue(
    //         new PutAlgae(m_lift,
    //             ConsLift.Pose.Put_A));
    // new JoystickButton(m_driveController,
    //     ConsController.Button.BUTTON_Y.id).onFalse(
    //         new PutAlgae(m_lift,
    //             ConsLift.Pose.Put_A));
    // new JoystickButton(m_driveController,
    //     ConsController.Button.BUTTON_A.id).onTrue(
    //         new PutAlgae(m_lift,
    //             ConsLift.Pose.Put_a));
    // new JoystickButton(m_driveController,
    //     ConsController.Button.BUTTON_A.id).onFalse(
    //         new PutAlgae(m_lift,
    //             ConsLift.Pose.Put_a));
    new JoystickButton(m_driveController,
        ConsController.Button.BUTTON_Y.id).toggleOnTrue(
            new PutCoral(m_lift,
                ConsLift.Pose.L4));
    new JoystickButton(m_driveController,
        ConsController.Button.BUTTON_A.id).toggleOnTrue(
            new PutCoral(m_lift,
                ConsLift.Pose.RESET_C));

  }

  private double getDriveControllerAxisOnDeadBand(double value, int power) {
    boolean isValueNegtive = value < 0;
    value = Math.abs(value) > ConsController.DEADBAND ? Math.abs(value) - ConsController.DEADBAND : 0;
    value = Math.pow(value, power);
    value = Tools.map(value, 0, Math.pow(1 - ConsController.DEADBAND, power), 0, 1);
    return isValueNegtive ? -value : value;
  }

  public void namedCommand() {

    NamedCommands.registerCommand("AlignR1", new AutoTag(m_driveTrain, ConsAuto.PositionName.r1));
    NamedCommands.registerCommand("AlignR3", new AutoTag(m_driveTrain, ConsAuto.PositionName.r3));
    NamedCommands.registerCommand("AlignR5", new AutoTag(m_driveTrain, ConsAuto.PositionName.r5));
    NamedCommands.registerCommand("AlignR6", new AutoTag(m_driveTrain, ConsAuto.PositionName.r6));
    NamedCommands.registerCommand("AlignR7", new AutoTag(m_driveTrain, ConsAuto.PositionName.r7));
    NamedCommands.registerCommand("AlignR8", new AutoTag(m_driveTrain, ConsAuto.PositionName.r8));
    NamedCommands.registerCommand("AlignR9", new AutoTag(m_driveTrain, ConsAuto.PositionName.r9));
    NamedCommands.registerCommand("AlignR10", new AutoTag(m_driveTrain, ConsAuto.PositionName.r10));
    NamedCommands.registerCommand("AlignR11", new AutoTag(m_driveTrain, ConsAuto.PositionName.r11));
    NamedCommands.registerCommand("AlignR12", new AutoTag(m_driveTrain, ConsAuto.PositionName.r12));
    NamedCommands.registerCommand("R10andR11", new AutoTagToTag(m_driveTrain, ConsAuto.PositionName.Rr));

    NamedCommands.registerCommand("B3", new AutoTag(m_driveTrain, ConsAuto.PositionName.B3));
    NamedCommands.registerCommand("B2", new AutoTag(m_driveTrain, ConsAuto.PositionName.B2));
    NamedCommands.registerCommand("B1", new AutoTag(m_driveTrain, ConsAuto.PositionName.B1));

    NamedCommands.registerCommand("Lock", new ZeroSpeed(m_driveTrain));

    NamedCommands.registerCommand("get coral", new GetCoral(m_intake, m_lift));
    NamedCommands.registerCommand("L3", new AutoPutCoral(m_lift, ConsLift.Pose.L3));
    NamedCommands.registerCommand("L4", new AutoPutCoral(m_lift, ConsLift.Pose.L4));
    NamedCommands.registerCommand("NG", new AutoPutCoral(m_lift, ConsLift.Pose.RESET_C));

    NamedCommands.registerCommand( "GetCoral", new AutoGetCoral(m_intake, m_lift));
    NamedCommands.registerCommand("YOLO",new CoralAlign( m_yolo, m_driveTrain));
  }
}