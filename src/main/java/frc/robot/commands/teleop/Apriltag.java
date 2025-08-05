package frc.robot.commands.teleop;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Tools;
// import frc.robot.constants.ConsAuto;
// import frc.robot.constants.ConsController;
// import frc.robot.constants.ConsVision;
// import java.util.ArrayList;
// import java.util.function.Supplier;

// import org.photonvision.PhotonUtils;

// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.VirtualPose;
// // import frc.robot.subsystems.Vision;
// import frc.robot.constants.ConsSwerve;

// public class Apriltag extends Command {
//   private DriveTrain m_driveTrain;
//   // private Vision m_vision;
//   protected VirtualPose m_VirtualPose;

//   private Supplier<Boolean> r1;
//   private Supplier<Boolean> r2;
//   private Supplier<Boolean> r3; 
//   private Supplier<Double> r4; 
//   private Supplier<Double> r5; 
//   private Supplier<Double> r6; 
//   private Supplier<Double> r7;
//   private Supplier<Double> r8;
//   private Supplier<Double> r9;
//   private Supplier<Boolean> r10;
//   private Supplier<Boolean> r11;
//   private Supplier<Boolean> r12;

//   private Supplier<Double> xAxis;
//   private Supplier<Double> yAxis;
//   private Supplier<Double> zAxis;
//   private Supplier<Boolean> isRed;
//   private ConsAuto.Position position;



// public Apriltag(DriveTrain m_driveTrain, VirtualPose m_VirtualPose,
// Supplier<Boolean> r1, 
// Supplier<Boolean> r2, 
// Supplier<Boolean> r3, 
// Supplier<Double> r4, 
// Supplier<Double> r5, 
// Supplier<Double> r6, 
// Supplier<Double> r7,
// Supplier<Double> r8,
// Supplier<Double> r9,
// Supplier<Boolean> r10,
// Supplier<Boolean> r11,
// Supplier<Boolean> r12,
// Supplier<Double> xAxis,
// Supplier<Double> yAxis,
// Supplier<Double> zAxis,
// Supplier<Boolean> isRed) {
//  this.m_driveTrain = m_driveTrain;
//  this.m_VirtualPose = m_VirtualPose;
//  this.r1 = r1;
//  this.r2 = r2;
//  this.r3 = r3;
//  this.r4 = r4;
//  this.r5 = r5;
//  this.r6 = r6;
//  this.r7 = r7;
//  this.r8 = r8;
//  this.r9 = r9;
//  this.r10 = r10;
//  this.r11 = r11;
//  this.r12 = r12;
//  this.xAxis = xAxis;
//  this.yAxis = yAxis;
//  this.zAxis = zAxis;
//  this.isRed = isRed;
//  addRequirements(m_driveTrain, m_VirtualPose);
// }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     ConsAuto.setAllianceColor(isRed.get());
//   }
  
  

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double x=m_driveTrain.getPose().getX(), y=m_driveTrain.getPose().getY();
//     if(r1.get()){
//       position = ConsAuto.pos("r1");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else if(r2.get()){
//       position = ConsAuto.pos("r2");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else if(r3.get()){
//       position = ConsAuto.pos("r3");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else if(r4.get()==1){
//       position = ConsAuto.pos("r4");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else if(r5.get()==1){
//       position = ConsAuto.pos("r5");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else if(r6.get()==-1){
//       position = ConsAuto.pos("r6");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else if(r7.get()==1){
//       position = ConsAuto.pos("r7");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else if(r8.get()==1){
//       position = ConsAuto.pos("r8");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else if(r9.get()==-1){
//       position = ConsAuto.pos("r9");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else if(r10.get()){
//       position = ConsAuto.pos("r10");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else if(r11.get()){
//       position = ConsAuto.pos("r11");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else if(r12.get()){
//       position = ConsAuto.pos("r12");
//       m_driveTrain.driveApriltag(x, y, position);
//     }else{
//       m_driveTrain.drive(
//         -getDriveControllerAxisOnDeadBand(xAxis.get(), 2)*ConsSwerve.throttleMaxSpeed,
//          getDriveControllerAxisOnDeadBand(yAxis.get(), 2)*ConsSwerve.throttleMaxSpeed,
//          getDriveControllerAxisOnDeadBand(zAxis.get(), 2)*ConsSwerve.kMaxRotationSpeed
//       );  
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   private double getDriveControllerAxisOnDeadBand(double value, int power) {
//     Boolean isValueNegtive = value < 0;
//     value = Math.abs(value) > ConsController.DEADBAND ? Math.abs(value) - ConsController.DEADBAND : 0;
//     value = Math.pow(value, power);
//     value = Tools.map(value, 0, Math.pow(1 - ConsController.DEADBAND, power), 0, 1);
//     return isValueNegtive ? -value : value;
//   } 
// }  
