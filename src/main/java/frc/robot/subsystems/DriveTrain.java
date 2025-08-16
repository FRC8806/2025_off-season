package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConsSwerve;

public class DriveTrain extends SubsystemBase {
  public AHRS m_gyro;
  private SwerveModule moduleA, moduleB, moduleC, moduleD;
  private SwerveDriveKinematics kinematics;

  private SwerveModulePosition[] previousPositions = null;
  private double totalDistanceMeters = 0.0;
  
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field = new Field2d();

  public RobotConfig robotConfig;

  // PathPlanner target rotation tracking
  private volatile Double ppTargetDeg = null;
  private volatile double ppTargetTs = 0.0;

  public DriveTrain(Supplier<Boolean> isRedAlliance) {

    m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);

    moduleA = new SwerveModule(ConsSwerve.Modules.A);
    moduleB = new SwerveModule(ConsSwerve.Modules.B);
    moduleC = new SwerveModule(ConsSwerve.Modules.C);
    moduleD = new SwerveModule(ConsSwerve.Modules.D);

    kinematics = new SwerveDriveKinematics(
        ConsSwerve.Modules.A.pos,
        ConsSwerve.Modules.B.pos,
        ConsSwerve.Modules.C.pos,
        ConsSwerve.Modules.D.pos
    );

    SmartDashboard.putData("field", field);

    // === PoseEstimator init ===
    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        getDiretion(),
        getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(0.02, 0.02, Math.toRadians(1.0)),
        VecBuilder.fill(0.7, 0.7, Math.toRadians(8.0))
    );

    // Continuous input for heading PID
    // vz.enableContinuousInput(-Math.PI, Math.PI);

    // === PathPlanner config ===
    ModuleConfig moduleConfig = new ModuleConfig(
        ConsSwerve.kWheelDiameter / 2.0,
        5.5,
        2,
        DCMotor.getKrakenX60(1),
        ConsSwerve.gearRatio_L3plus,
        85,
        1
    );

    Translation2d fl = ConsSwerve.Modules.A.pos;
    Translation2d fr = ConsSwerve.Modules.B.pos;
    Translation2d bl = ConsSwerve.Modules.C.pos;
    Translation2d br = ConsSwerve.Modules.D.pos;

    robotConfig = new RobotConfig(
        67.0,
        5.937,
        moduleConfig,
        fl, fr, bl, br
    );

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,                         // resets estimator
        this::getRobotRelativeSpeeds,            // robot-relative
        (speeds, feedforwards) -> this.autoDrive(speeds),
        new PPHolonomicDriveController(
            ConsSwerve.TRANSLATION_PID_CONSTANTS,
            ConsSwerve.ROTATION_PID_CONSTANTS
        ),
        robotConfig,
        () -> isRedAlliance.get(),
        this
    );

    // Log PP target pose so we can reuse its target heading when needed
    PathPlannerLogging.setLogTargetPoseCallback(pose -> {
      ppTargetDeg = pose.getRotation().getDegrees();
      ppTargetTs = Timer.getFPGATimestamp();
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
  }

  @Override
  public void periodic() {
    // Predict with wheel odometry + gyro
    poseEstimator.update(getDiretion(), getModulePositions());

    Pose2d pose = getPose();
    SmartDashboard.putNumber("Robot X", pose.getX());
    SmartDashboard.putNumber("Robot Y", pose.getY());
    SmartDashboard.putNumber("Robot Heading (deg)", pose.getRotation().getDegrees());

    // Rough distance tracking
    SwerveModulePosition[] currentPositions = getModulePositions();
    if (previousPositions != null) {
      double distanceSum = 0.0;
      for (int i = 0; i < currentPositions.length; i++) {
        double delta = currentPositions[i].distanceMeters - previousPositions[i].distanceMeters;
        distanceSum += Math.abs(delta);
      }
      totalDistanceMeters += distanceSum / currentPositions.length;
    }
    previousPositions = currentPositions;
    SmartDashboard.putNumber("Total Distance (m)", totalDistanceMeters);

    field.setRobotPose(pose);
  }

  // ================= PathPlanner hooks =================

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getDiretion(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
      moduleA.getState(), moduleB.getState(), moduleC.getState(), moduleD.getState()
    };
  }

  public void autoDrive(ChassisSpeeds speeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    // targetSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(targetSpeeds, getDiretion());
    targetSpeeds = new ChassisSpeeds(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, -targetSpeeds.omegaRadiansPerSecond);
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  // ================= Manual driving =================

  public void drive(double xSpeed, double ySpeed, double rSpeed) {
    drive(new ChassisSpeeds(xSpeed, ySpeed, rSpeed));
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, getDiretion());
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, ConsSwerve.throttleMaxSpeed);
    moduleA.setState(states[0]);
    moduleB.setState(states[1]);
    moduleC.setState(states[2]);
    moduleD.setState(states[3]);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[]{
      moduleA.getPosition(), moduleB.getPosition(), moduleC.getPosition(), moduleD.getPosition()
    };
  }

  public Rotation2d getDiretion() {
    return m_gyro.getRotation2d();
  }

  /** Current fused pose (odometry + vision). */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void stopModules() {
    drive(new ChassisSpeeds());
  }

  // ===== Vision integration =====

  /**
   * Inject a time-stamped vision measurement with dynamic std devs.
   * @param visionPose Pose2d in field coordinates from camera solver
   * @param timestampSeconds Photon result timestamp (image time), seconds
   * @param std dynamic standard deviations (m, m, rad)
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Vision.VisionStdDevs std) {
    Matrix<N3, N1> measStd = VecBuilder.fill(std.stdX, std.stdY, std.stdThetaRad);
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, measStd);
  }

  // ===== Reset helpers =====

  /** Reset estimator to the given pose. Prefer this over any direct odometry reset. */
  public void updatePose(Pose2d pose) {
    poseEstimator.resetPosition(getDiretion(), getModulePositions(), pose);
  }

  /** Reset estimator to the given pose. */
  public void updatePose(double x, double y, double degrees) {
    Pose2d newPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    poseEstimator.resetPosition(getDiretion(), getModulePositions(), newPose);
  }
}
