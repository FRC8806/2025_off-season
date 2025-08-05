package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collector;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Tools;
import frc.robot.constants.ConsAuto;
import frc.robot.constants.ConsSwerve;
// import frc.robot.subsystems.Vision;
import java.util.Arrays;
import java.util.stream.Collectors;
import frc.robot.constants.ConsAuto.Position;

import com.pathplanner.lib.config.ModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveTrain extends SubsystemBase {
  public AHRS m_gyro;
  private SwerveModule moduleA;
  private SwerveModule moduleB;
  private SwerveModule moduleC;
  private SwerveModule moduleD;

  private SwerveModule[] swerveModules;
  private SwerveDriveKinematics kinematics;
  // private SwerveDriveOdometry odometry;

  private SwerveModulePosition[] previousPositions = null;
  private double totalDistanceMeters = 0.0;

  private final AprilTagFieldLayout fieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();

  private SwerveDriveOdometry odometry;
  // private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
  // Rotation2d.fromDegrees(-m_gyro.getAngle()),
  // getModulePositions());

  // private PIDController vx = new PIDController(2.8 , 0 , 0.4);//1.2,0.3,0.1 1 0
  // 0.2
  // private PIDController vy = new PIDController(2.8 , 0 ,
  // 0.4);//vx同vy//3.1,0,0.6
  // public PIDController vz = new PIDController(0.05,0 ,0.0);//0.005, 0, 0.0005

  private Supplier<Boolean> isRedAlliance;
  private Field2d field = new Field2d();
  public RobotConfig robotConfig;

  /** Creates a new DriveTrain. */

  // private final SwerveModule[] swerveModules = { moduleA, moduleB, moduleC,
  // moduleD };

  public DriveTrain(Supplier<Boolean> isRedAlliance) {
    // public DriveTrain(Supplier<Boolean> isRedAlliance) {
    // vz.enableContinuousInput(-180, 180);
    this.isRedAlliance = isRedAlliance;
    m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
    moduleA = new SwerveModule(ConsSwerve.Modules.A);
    moduleB = new SwerveModule(ConsSwerve.Modules.B);
    moduleC = new SwerveModule(ConsSwerve.Modules.C);
    moduleD = new SwerveModule(ConsSwerve.Modules.D);

    swerveModules = new SwerveModule[] { moduleA, moduleB, moduleC, moduleD };

    kinematics = new SwerveDriveKinematics(
        ConsSwerve.Modules.A.pos,
        ConsSwerve.Modules.B.pos,
        ConsSwerve.Modules.C.pos,
        ConsSwerve.Modules.D.pos);
    SmartDashboard.putData("field", field);

    odometry = new SwerveDriveOdometry(
        kinematics,
        getDiretion(), // m_gyro.getRotation2d(),
        getModulePositions());

    // vz.enableContinuousInput(-180, 180);
    SmartDashboard.putData("field", field);
    ModuleConfig moduleConfig = new ModuleConfig(
        ConsSwerve.kWheelDiameter / 2.0, // Wheel radius in meters
        5.5, // Max theoretical velocity in m/s (estimate based on gearing)
        1.19, // Coefficient of friction (typical = 1.0 to 1.3)
        DCMotor.getKrakenX60(1), // Drive motor type (e.g. 1 Falcon 500)
        ConsSwerve.gearRatio_L3plus,
        40.0, // Drive current limit in Amps
        1 // Number of motors per module
    );

    Translation2d frontLeftOffset = ConsSwerve.Modules.A.pos;
    Translation2d frontRightOffset = ConsSwerve.Modules.B.pos;
    Translation2d backLeftOffset = ConsSwerve.Modules.C.pos;
    Translation2d backRightOffset = ConsSwerve.Modules.D.pos;

    // RobotConfig using the moduleConfig and offsets
    RobotConfig robotConfig = new RobotConfig(
        67.0, // Robot mass in kg
        6.0, // Moment of Inertia (tune for realistic path following)
        moduleConfig, // The ModuleConfig object defined above
        frontLeftOffset,
        frontRightOffset,
        backLeftOffset,
        backRightOffset);

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::autoDrive, // Method that will drive the robot given ROBOT RELATIVE
                         // ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            ConsSwerve.TRANSLATION_PID_CONSTANTS, // Translation
                                                  // PID
                                                  // constants
            ConsSwerve.ROTATION_PID_CONSTANTS // Rotation PID
                                              // constants
        ),
        robotConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

  }

  @Override
  public void periodic() {
    odometry.update(m_gyro.getRotation2d(), getModulePositions());

    Pose2d currentPose = getPose();
    SmartDashboard.putNumber("Robot X", currentPose.getX());
    SmartDashboard.putNumber("Robot Y", currentPose.getY());

    // This method will be called once per scheduler run

    SwerveModulePosition[] currentPositions = getModulePositions();

    if (previousPositions != null) {
      double distanceSum = 0.0;

      for (int i = 0; i < currentPositions.length; i++) {
        double delta = currentPositions[i].distanceMeters - previousPositions[i].distanceMeters;
        distanceSum += Math.abs(delta);
      }
      double averageDelta = distanceSum / currentPositions.length;
      totalDistanceMeters += averageDelta;
    }
    previousPositions = currentPositions;
    SmartDashboard.putNumber("Total Distance", totalDistanceMeters);
    SmartDashboard.putNumber("a", moduleA.m_encoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("b", moduleB.m_encoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("c", moduleC.m_encoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("d", moduleD.m_encoder.getAbsolutePosition().getValueAsDouble());

    SmartDashboard.putNumber("z", getPose().getRotation().getDegrees());
    field.setRobotPose(getPose());
  }

  // public void testZControl(double current, double target) {
  // double output = vz.calculate(current, target);
  // SmartDashboard.putNumber("zTestOut", output);
  // }
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(getDiretion(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        moduleA.getState(),
        moduleB.getState(),
        moduleC.getState(),
        moduleD.getState()
    };
  }

  public void autoDrive(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    targetSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getDiretion());
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public void drive(double xSpeed, double ySpeed, double rSpeed) {
    drive(new ChassisSpeeds(xSpeed, ySpeed, rSpeed));
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getDiretion());
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public void drive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond,
      Rotation2d latestEstimatedPose) {
    // 將場地座標轉換為機器人相對座標，根據目前車頭朝向 (getDirection)
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vxMetersPerSecond,
        vyMetersPerSecond,
        omegaRadiansPerSecond,
        latestEstimatedPose// 你的 getDirection() 方法會提供目前朝向（Rotation2d）
    );

    // 使用現有函式處理速度拆解與模組設定
    drive(fieldRelativeSpeeds);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, ConsSwerve.throttleMaxSpeed);
    moduleA.setState(states[0]);
    moduleB.setState(states[1]);
    moduleC.setState(states[2]);
    moduleD.setState(states[3]);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        moduleA.getPosition(),
        moduleB.getPosition(),
        moduleC.getPosition(),
        moduleD.getPosition()
    };
  }

  public Rotation2d getDiretion() {
    return m_gyro.getRotation2d();
  }

  public List<Pose2d> getFieldAprilTagPoses() {
    return fieldLayout.getTags().stream().map(tag -> tag.pose.toPose2d()).collect(Collectors.toList());
  }

  public double getCurrentVelocityX() {
    double vx = 0.0;

    for (SwerveModule module : swerveModules) {
      SwerveModuleState state = module.getState();
      vx += state.speedMetersPerSecond * state.angle.getCos(); // x 分量
    }

    return vx / swerveModules.length;
  }

  // 計算整體底盤在 Y 方向的平均移動速度 (m/s)
  public double getCurrentVelocityY() {
    double vy = 0.0;

    for (SwerveModule module : swerveModules) {
      SwerveModuleState state = module.getState();
      vy += state.speedMetersPerSecond * state.angle.getSin(); // y 分量
    }

    return vy / swerveModules.length;
  }

  public Rotation2d getYaw() {
    return getDiretion(); // 假設你用的是 Pigeon、NavX 等感測器
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void stopModules() {
    drive(new ChassisSpeeds());
  }

  // public void driveApriltag(double x, double y, ConsAuto.Position position) {
  //   double errorX = Math.abs(x - position.x);
  //   double errorY = Math.abs(y - position.y);
  //   double xOutput = vx.calculate(x, position.x);
  //   double yOutput = vy.calculate(y, position.y);

  //   if (errorX < 0.02)
  //     xOutput = 0;
  //   if (errorY < 0.02)
  //     yOutput = 0;

  //   xOutput = Tools.highSpeed(xOutput, 0.8);
  //   yOutput = Tools.highSpeed(yOutput, 0.8);

  //   SmartDashboard.putNumber("x-target", position.x);
  //   SmartDashboard.putNumber("y-target", position.y);
  //   SmartDashboard.putNumber("z-target", position.z);
  //   SmartDashboard.putNumber("x-now", x);
  //   SmartDashboard.putNumber("y-now", y);
  //   SmartDashboard.putNumber("z-now", getPose().getRotation().getDegrees());

  //   Rotation2d currentRot = getPose().getRotation();
  //   Rotation2d targetRot = Rotation2d.fromDegrees(position.z);
  //   double angleError = targetRot.minus(currentRot).getDegrees();
  //   double zOutput = vz.calculate(0, -angleError);

  //   zOutput = Tools.highSpeed(zOutput, 0.3);
  //   if (Math.abs(angleError) < 1.5)
  //     zOutput = 0;

  //   ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
  //       new ChassisSpeeds(xOutput, yOutput, zOutput),
  //       getPose().getRotation());

  //   SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
  //   setModuleStates(states);
  // }

  public void updatePose(Pose2d pose) {
    // odometry.resetPose(pose);
    // // odometry.resetTranslation(tl);
    // public void updatePose(Pose2d pose) {
    odometry.resetPosition(getDiretion(), getModulePositions(), pose);
  }

  public void updatePose(double x, double y, double degrees) {
    Pose2d newPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    odometry.resetPosition(getDiretion(), getModulePositions(), newPose);
  }
  // public void updatePose(double x, double y, double degrees) {
  // // TODO Auto-generated method stub
  // throw new UnsupportedOperationException("Unimplemented method 'updatePose'");
  // }
}