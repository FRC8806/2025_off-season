package frc.robot.commands.teleop;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Vision;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.SwerveModule;
// import frc.robot.subsystems.VirtualPose;
// import frc.robot.constants.ConsSwerve;

// public class Tracking extends Command {

//     private DriveTrain m_driveTrain;
//     private  Vision vision;
//     private VirtualPose m_VirtualPose;
//     public  double throttleMaxSpeed = ConsSwerve.throttleMaxSpeed;
//     private  double kMaxRotationSpeed = ConsSwerve.kMaxRotationSpeed; 

//     // 馬達和機械參數計算出來的最大速度（m/s）
  

//     // PID 控制器 (比例參數可再調整)
//     private final PIDController pidX = new PIDController(1.0, 0.0, 0.0);
//     private final PIDController pidY = new PIDController(1.0, 0.0, 0.0);
//     private final PIDController pidRot = new PIDController(3.0, 0.0, 0.0);

//     // 限速器限制加速度 (避免突變)
//     private final SlewRateLimiter vxLimiter = new SlewRateLimiter(3.0);  // m/s²
//     private final SlewRateLimiter vyLimiter = new SlewRateLimiter(3.0);
//     private final SlewRateLimiter rotLimiter = new SlewRateLimiter(5.0); // rad/s²

//     private final double distanceTolerance = 0.1;      // 距離小於此視為到位
//     private final double slowDownDistance = 0.5;     // 接近時開始減速



    


//     public Tracking(DriveTrain m_driveTrain, Vision vision) {
//         this.m_driveTrain = m_driveTrain;
//         this.vision = vision;
//         addRequirements(m_driveTrain, vision);

//         // PID 容忍度設定
//         pidX.setTolerance(0.05);       // 速度容忍度 m/s
//         pidY.setTolerance(0.05);
//         pidRot.setTolerance(Math.toRadians(1));      // 旋轉容忍度 1度
//     }



//     public void initialize() {
        
//         // SmartDashboard.putNumber("TrackingCoral/Distance", distance);
//         // SmartDashboard.putNumber("TrackingCoral/AngleOffset", angleOffset);
//         // SmartDashboard.putNumber("TrackingCoral/Yaw", yaw);
//         // SmartDashboard.putNumber("TrackingCoral/VxTarget", vxTarget);
//         // SmartDashboard.putNumber("TrackingCoral/VyTarget", vyTarget);
//         // SmartDashboard.putNumber("TrackingCoral/VxOutput", vxLimited);
//         // SmartDashboard.putNumber("TrackingCoral/VyOutput", vyLimited);
//         // SmartDashboard.putNumber("TrackingCoral/RotationOutput", rotLimited);
//     }
//     @Override
//     public void execute() {

//         double distance = m_VirtualPose.getTargetDistance();//要從dashboard上抓//直線距離
//         double angleOffset = m_VirtualPose.getTargetAngle();//要從dashboard上抓  //角度偏差
//         // double yaw = m_driveTrain.getYaw().getDegrees();//卡
//         double yaw = m_driveTrain.getPose().getRotation().getDegrees();

//         yaw = (yaw + 360) % 360;// 確保 yaw 介於0~360度


//         double maxSpeed = throttleMaxSpeed;
//         double speed = (distance > slowDownDistance) ? maxSpeed : maxSpeed * (distance / slowDownDistance);

//         // 四象限速度轉換，取得目標速度向量 (vxTarget, vyTarget)
//         double[] velocityTargets = convertRelativeToFieldCoordinates(yaw, angleOffset, speed);
//         double vxTarget = velocityTargets[0];
//         double vyTarget = velocityTargets[1];

//         // 取得目前底盤速度，假設你有實作這兩個方法
//         double currentVx = m_driveTrain.getCurrentVelocityX();
//         double currentVy = m_driveTrain.getCurrentVelocityY();

//         // PID 計算平移速度修正輸出
//         double vxOutput = pidX.calculate(currentVx, vxTarget);
//         double vyOutput = pidY.calculate(currentVy, vyTarget);

//         // 限制合成速度不超過最大速度
//         double speedRaw = Math.hypot(vxOutput, vyOutput);
//         //double yaw = (yaw % 360 + 360) % 360;
//         double vxLimited = vxLimiter.calculate(vxOutput);
//         double vyLimited = vyLimiter.calculate(vyOutput);
//         double rotOutput = pidRot.calculate(angleOffset, 0);
//         double rotLimited = rotLimiter.calculate(rotOutput);
       
//         if (Double.isNaN(distance) || Double.isNaN(angleOffset)) {
//             m_driveTrain.stopModules();
//             pidX.reset();
//             pidY.reset();
//             pidRot.reset();
//             return;
//         }

//         // 根據距離線性減速，距離大於 slowDownDistance 時用最大速度
        
//         if (speedRaw > maxSpeed) {
//             double scale = maxSpeed / speedRaw;
//             vxOutput *= scale;
//             vyOutput *= scale;
//         }        

//         // 限制旋轉速度在最大旋轉速度範圍
//         if (rotOutput > kMaxRotationSpeed) rotOutput = kMaxRotationSpeed;
//         else if (rotOutput < -kMaxRotationSpeed) rotOutput = -kMaxRotationSpeed;

//         // 加速限制，避免速度跳變
        
       

//         // 發送速度指令到底盤
//         m_driveTrain.drive(vxLimited, vyLimited, rotLimited, Rotation2d.fromDegrees(yaw)); 
//     }

//     @Override
//     public boolean isFinished() {
//         // double distance = vision.getTargetDistance();
//         // // 距離小於容忍值即完成
//         // return !Double.isNaN(distance) && distance < distanceTolerance;
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_driveTrain.stopModules();
//         pidX.reset();
//         pidY.reset();
//         pidRot.reset();
//     }

//     /**
//      * 四象限速度方向轉換
//      * @param yaw 車頭角度（0~360度）
//      * @param angleOffset 目標相對車頭角度（度）
//      * @param speed 目標速度大小(m/s)
//      * @return double[] {vx, vy} 場地座標速度向量
//      */
//     private double[] convertRelativeToFieldCoordinates(double yaw, double angleOffset, double speed) {
//         int quadrant = (int) (yaw / 90) + 1;

//         double angleRad = Math.toRadians(angleOffset);
//         double vx = 0;
//         double vy = 0;

//         switch (quadrant) {
//             case 1: // 0~90度，x負，y正
//                 vx = -speed * Math.cos(angleRad);
//                 vy = speed * Math.sin(angleRad);
//                 break;
//             case 2: // 90~180度，x正，y正
//                 vx = speed * Math.cos(angleRad);
//                 vy = speed * Math.sin(angleRad);
//                 break;
//             case 3: // 180~270度，x負，y負
//                 vx = -speed * Math.cos(angleRad);
//                 vy = -speed * Math.sin(angleRad);
//                 break;
//             case 4: // 270~360度，x正，y負
//                 vx = speed * Math.cos(angleRad);
//                 vy = -speed * Math.sin(angleRad);
//                 break;
//             default:
//                 vx = speed * Math.cos(angleRad);
//                 vy = speed * Math.sin(angleRad);
//                 break;
//         }
//         return new double[]{vx, vy};
//     }
// }

