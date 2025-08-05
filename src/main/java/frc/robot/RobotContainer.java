package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.AutoGetCoral;
import frc.robot.commands.auto.AutoPutCoral;
import frc.robot.commands.auto.AutoTag;
import frc.robot.commands.teleop.AllDefaultCommand;
import frc.robot.commands.teleop.AprilTag2;
import frc.robot.commands.teleop.ClimberDefaultCommand;
import frc.robot.commands.teleop.Climberup;
import frc.robot.commands.teleop.Climberupup;
import frc.robot.commands.teleop.GetAlgae;
import frc.robot.commands.teleop.GetCoral1;
import frc.robot.commands.teleop.GetCoral2;
import frc.robot.commands.teleop.L1;
import frc.robot.commands.teleop.Outputcoral;
import frc.robot.commands.teleop.PutAlgae;
import frc.robot.commands.teleop.PutCoral;
import frc.robot.constants.ConsAuto;
// import frc.robot.commands.Tracking;
import frc.robot.constants.ConsController;
import frc.robot.constants.ConsLift;
import frc.robot.constants.ConsSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
//import frc.robot.subsystems.VirtualPose;
import edu.wpi.first.wpilibj2.command.WaitCommand;
 import frc.robot.subsystems.Vision;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  public final Intake m_intake = new Intake();
   //public final VirtualPose m_VirtualPose;

  private final DriveTrain m_driveTrain;
   public final Vision m_vision;
  public final Lift m_lift = new Lift();
  public final Climber m_climber = new Climber();
  
   
  public final XboxController m_driveController = new XboxController(ConsController.kDriveControllerPort);
  public final XboxController m_operatorController = new XboxController(ConsController.kOperatorControllerPort);
  public final XboxController m_buttonBroadController = new XboxController(ConsController.kButtonBroadControllerPort);

  public Supplier<Boolean> isRedAliance;
    private SendableChooser<Command> autoChooser = new SendableChooser<>();




  public RobotContainer(Supplier<Boolean> isRedAliance) {
    m_driveTrain = new DriveTrain(isRedAliance);
     m_vision = new Vision(m_driveTrain);
    //m_VirtualPose = new VirtualPose(m_driveTrain);
    this.isRedAliance = isRedAliance;
    configureBindings();
    setDefaultCommand();
    namedCommand();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // SmartDashboard.putData("PATH",createDynamicPathCommand(m_driveTrain));


      // m_driveTrain.setDefaultCommand(new RunCommand(() -> m_driveTrain.drive(
      //     () -> -getDriveControllerAxisOnDeadBand(ConsController.Axis.LEFT_STICK_Y.id, 2) * ConsSwerve.throttleMaxSpeed,
      //     () -> -getDriveControllerAxisOnDeadBand(ConsController.Axis.LEFT_STICK_X.id, 2) * ConsSwerve.throttleMaxSpeed,
      //     () -> -getDriveControllerAxisOnDeadBand(ConsController.Axis.RIGHT_STICK_X.id, 2)
      //         * ConsSwerve.kMaxRotationSpeed), m_driveTrain ));

      // Configure the trigger bindings
    
    }
  
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
      }

  private void setDefaultCommand(){
    //for test
    // m_driveTrain.setDefaultCommand(new AlignToTag(m_driveTrain, vision, 
    // ()-> m_ButtonBroadController.getAButton(), ()-> m_ButtonBroadController.getXButton(), ()-> m_ButtonBroadController.get));
    // m_intake.setDefaultCommand(new AllDefaultCommand(m_intake, m_lift,
    // () -> m_driveController.getLeftTriggerAxis(), () -> m_operatorController.getRightTriggerAxis(), 
    // () -> m_operatorController.getLeftBumperButton(), () -> m_operatorController.getRightBumperButton(),
    // () -> m_operatorController.getRightY(), () -> m_operatorController.getLeftY(), () -> m_operatorController.getAButton()));
    
    //  m_climber.setDefaultCommand(new ClimberDefaultCommand(m_climber, 
    // () -> m_driveController.getRightBumperButton(), 
    // () -> m_driveController.getLeftBumperButton()
    // ));

    // m_driveTrain.setDefaultCommand(new Apriltag(m_driveTrain, m_VirtualPose, 
    // ()->m_buttonBroadController.getYButton(), ()->m_buttonBroadController.getLeftBumperButton(), ()->m_buttonBroadController.getRightBumperButton(), 
    // ()->m_buttonBroadController.getLeftTriggerAxis(), ()->m_buttonBroadController.getRightTriggerAxis(), 
    // ()->m_buttonBroadController.getLeftX(), ()->m_buttonBroadController.getLeftX(), 
    // ()->m_buttonBroadController.getLeftY(), ()->m_buttonBroadController.getLeftY(), 
    // ()->m_buttonBroadController.getAButton(), ()->m_buttonBroadController.getBButton(), ()->m_buttonBroadController.getXButton(), 
    // ()->m_driveController.getLeftX(), ()->m_driveController.getLeftY(), ()->m_driveController.getRightX(),
    // () -> isRedAliance.get()));

    m_driveTrain.setDefaultCommand(new AprilTag2(m_driveTrain, m_vision, 
    ()->m_buttonBroadController.getYButton(), ()->m_buttonBroadController.getLeftBumperButton(), ()->m_buttonBroadController.getRightBumperButton(), 
    ()->m_buttonBroadController.getLeftTriggerAxis(), ()->m_buttonBroadController.getRightTriggerAxis(), 
    ()->m_buttonBroadController.getLeftX(), ()->m_buttonBroadController.getLeftX(), 
    ()->m_buttonBroadController.getLeftY(), ()->m_buttonBroadController.getLeftY(), 
    ()->m_buttonBroadController.getAButton(), ()->m_buttonBroadController.getBButton(), ()->m_buttonBroadController.getXButton(), 
    ()->m_driveController.getLeftX(), ()->m_driveController.getLeftY(), ()->m_driveController.getRightX(),
    () -> isRedAliance.get()));

    // m_intake.setDefaultCommand(new AllDefaultCommand(m_intake, m_lift,
    // () -> m_operatorController.getLeftTriggerAxis(), () -> m_operatorController.getRightTriggerAxis(), 
    // () -> m_operatorController.getLeftBumperButton(), () -> m_operatorController.getRightBumperButton(),
    // () -> m_operatorController.getRightY(), () -> m_operatorController.getLeftY(), () -> m_operatorController.getAButton()));

    m_climber.setDefaultCommand(new ClimberDefaultCommand(m_climber, () -> m_operatorController.getStartButton(), () -> m_operatorController.getBackButton()));
    //  m_climber.setDefaultCommand(new ClimberDefaultCommand(m_climber, () -> m_driveController.getStartButton(), () -> m_driveController.getBackButton()));

  
  }


  private void configureBindings() {
//     new JoystickButton(m_operatorController, ConsController.Button.BUTTON_A.id).toggleOnTrue(new PutCoral(m_lift, ConsLift.Pose.RESET_C));
//     new JoystickButton(m_operatorController, ConsController.Button.BUTTON_B.id).toggleOnTrue(new PutCoral(m_lift, ConsLift.Pose.L3));
//     new JoystickButton(m_operatorController, ConsController.Button.BUTTON_Y.id).toggleOnTrue(new PutCoral(m_lift, ConsLift.Pose.L4));
//     new JoystickButton(m_operatorController, ConsController.Button.BUTTON_X.id).toggleOnTrue(new PutCoral(m_lift, ConsLift.Pose.L2));
//     new JoystickButton(m_operatorController, ConsController.Button.BUTTON_C.id).toggleOnTrue(new GetCoral2(m_lift));
// /*==================================================================================================================================================*/
//     new JoystickButton(m_driveController, ConsController.Button.BUTTON_START.id).onTrue(new Climberupup(m_lift,m_climber,ConsLift.Pose.climber));
//     new JoystickButton(m_driveController, ConsController.Button.BUTTON_BACK.id).onTrue(new Climberup(m_lift,m_climber,ConsLift.Pose.climber));
//     new JoystickButton(m_driveController, ConsController.Button.BUTTON_RB.id).onTrue(new GetCoral1(m_intake, m_lift));
//     new JoystickButton(m_driveController, ConsController.Button.BUTTON_RB.id).onFalse(new GetCoral1(m_intake, m_lift));
//     new JoystickButton(m_driveController, ConsController.Button.BUTTON_LB.id).onTrue(new Outputcoral( m_intake));
// /*====================================================================================================================================================*/
//     new JoystickButton(m_driveController, ConsController.Button.BUTTON_B.id).toggleOnTrue(new GetAlgae(m_lift,ConsLift.Pose.L3A));
//     new JoystickButton(m_driveController, ConsController.Button.BUTTON_X.id).toggleOnTrue(new GetAlgae(m_lift,ConsLift.Pose.L2A));
//     new JoystickButton(m_driveController, ConsController.Button.BUTTON_Y.id).onTrue(new PutAlgae(m_lift, ConsLift.Pose.Put_A));
//     new JoystickButton(m_driveController, ConsController.Button.BUTTON_Y.id).onFalse(new PutAlgae(m_lift, ConsLift.Pose.Put_A));









    new JoystickButton(m_driveController, ConsController.Button.BUTTON_RB.id).toggleOnTrue(new PutCoral(m_lift, ConsLift.Pose.RESET_C));
    new JoystickButton(m_driveController, ConsController.Button.BUTTON_B.id).toggleOnTrue(new PutCoral(m_lift, ConsLift.Pose.L3));
    new JoystickButton(m_driveController, ConsController.Button.BUTTON_Y.id).toggleOnTrue(new PutCoral(m_lift, ConsLift.Pose.L4));

    // //  new JoystickButton(m_driveController, ConsController.Button.BUTTON_X.id).toggleOnTrue(new PutCoral(m_lift, ConsLift.Pose.L1));
    
    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_X.id).toggleOnTrue(new PutCoral(m_lift, ConsLift.Pose.climber));
    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_A.id).toggleOnTrue((new PutCoral(m_lift, ConsLift.Pose.RESET_C)));
    // new JoystickButton(m_DriveController, ConsController.Button.BUTTON_A.id).onTrue(new GetCoral1(m_intake, m_lift));
    //  new JoystickButton(m_driveController, ConsController.Button.BUTTON_START.id).onTrue(new Climberupup(m_lift,m_climber,ConsLift.Pose.climber));
    //  new JoystickButton(m_driveController, ConsController.Button.BUTTON_BACK.id).onTrue(new Climberup(m_lift,m_climber,ConsLift.Pose.climber));

    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_RB.id).onTrue(new GetCoral2(m_lift));
    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_B.id).toggleOnTrue(new GetAlgae(m_lift,ConsLift.Pose.L3A));
    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_X.id).toggleOnTrue(new GetAlgae(m_lift,ConsLift.Pose.L2A));
    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_Y.id).onTrue(new PutAlgae(m_lift, ConsLift.Pose.Put_A));
    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_Y.id).onFalse(new PutAlgae(m_lift, ConsLift.Pose.Put_A));
    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_X.id).toggleOnTrue(new PutCoral(m_lift, ConsLift.Pose.climber));
    // // new JoystickButton(m_operatorController, ConsController.Button.BUTTON_RB.id).toggleOnTrue(new PutCoral(m_lift, ConsLift.Pose.RESET));//

    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_A.id).toggleOnTrue(new GetCoral1(m_intake, m_lift));
        //new JoystickButton(m_driveController, ConsController.Button.BUTTON_A.id).onFalse(new GetCoral1(m_intake, m_lift));

    //  new JoystickButton(m_driveController, ConsController.Button.BUTTON_A.id).toggleOnTrue(new GetCoral2(m_lift));
    //  new JoystickButton(m_driveController, ConsController.Button.BUTTON_A.id).onTrue(new Outputcoral( m_intake));
    // new JoystickButton(m_driveController, ConsController.Button.BUTTON_A.id).onTrue(new Tracking(m_driveTrain, m_vision));

    // new JoystickButton(m_operatorController, ConsController.Button.BUTTON_LB.id).toggleOnTrue(new GetAlgae(m_lift, ConsLift.Pose.L2A));
    // new JoystickButton(m_operatorController, ConsController.Button.BUTTON_B.id).toggleOnTrue(new GetAlgae(m_lift, ConsLift.Pose.L3A));
  
    // new JoystickButton(m_DriveController, XboxController.Button.kA.value)
    // .onTrue(new Aligntotag(m_driveTrain, vision));//不知道要用哪個按鈕
        
    // B 鍵對齊到右邊（offsetSign = +1）
    // new JoystickButton(m_DriveController, XboxController.Button.kB.value)
    //     .onTrue(new Aligntotag(m_driveTrain, vision, 1, 0.5));//不知道要用哪個按鈕
  
  }


  // private Double getDriveControllerAxisOnDeadBand(int axisID, int power) {
  //   double value = m_driveController.getRawAxis(axisID);
  //   Boolean isValueNegtive = value < 0;
  //   value = Math.abs(value) > ConsController.DEADBAND ? Math.abs(value) - ConsController.DEADBAND : 0;
  //   value = Math.pow(value, power);
  //   value = Tools.map(value, 0, Math.pow(1 - ConsController.DEADBAND, power), 0, 1);
  //   return isValueNegtive ? -value : value;
  // }
  // public Command getAutonomousCommand() {
  //   return null;
  // }

  // Command createDynamicPathCommand(DriveTrain swerve) {
  //     // Define waypoints from poses
  //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
  //             new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
  //             new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
  //             new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
  //     );

  //     // Path constraints
  //     PathConstraints constraints = new PathConstraints(3.0, 1.0, 2 * Math.PI, 4 * Math.PI);

  //     // Create the path
  //     PathPlannerPath path = new PathPlannerPath(
  //             waypoints,
  //             constraints,
  //             null,
  //             new GoalEndState(0.0, Rotation2d.fromDegrees(-90))
  //     );

  //     // Prevent flipping
  //     path.preventFlipping = true;
  //     // Return a follow path command (replace with your drive subsystem)
  //     return AutoBuilder.followPath(path);
  // }
  public void namedCommand() {
    NamedCommands.registerCommand("AlignR6", new AutoTag(m_driveTrain, ConsAuto.getPosition(ConsAuto.PositionName.r6)));
    NamedCommands.registerCommand("AlignR7", new AutoTag(m_driveTrain, ConsAuto.getPosition(ConsAuto.PositionName.r7)));
    NamedCommands.registerCommand("AlignR8", new AutoTag(m_driveTrain, ConsAuto.getPosition(ConsAuto.PositionName.r8)));

    NamedCommands.registerCommand("get coral", new AutoGetCoral(m_intake, m_lift));
    NamedCommands.registerCommand("L2", new AutoPutCoral(m_lift, ConsLift.Pose.L2));
    NamedCommands.registerCommand("L3", new AutoPutCoral(m_lift, ConsLift.Pose.L3)); 
    NamedCommands.registerCommand("L4", new AutoPutCoral(m_lift, ConsLift.Pose.L4));
    NamedCommands.registerCommand("reset coral", new AutoPutCoral(m_lift, ConsLift.Pose.RESET_C));
  }
}