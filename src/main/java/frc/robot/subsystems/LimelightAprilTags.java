// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class LimelightAprilTags extends SubsystemBase {
//     static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-new");

//   public int pipelineindex = 0;
//   public static double tx, ty, tid, tv, tz;
//   public double[] result;
//   public Translation3d translation3d;
//   public Rotation3d rotation3d;
//   public Pose2d botPose2d;
//   public double ledStatus = 1;

//   /** Creates a new LimelightAprilTags. */
//   public LimelightAprilTags() {
//     table.getEntry("ledMode").setNumber(1);
//     NetworkTableInstance.getDefault().getTable("limelight-new").getEntry("pipeline").setNumber(pipelineindex);
//   }

//   @Override
//   public void periodic() {
//     tv = NetworkTableInstance.getDefault().getTable("limelight-new").getEntry("tv").getDouble(0.0);
//     tid = NetworkTableInstance.getDefault().getTable("limelight-new").getEntry("tid").getDouble(-1.0);
//     tx = NetworkTableInstance.getDefault().getTable("limelight-new").getEntry("tx").getDouble(0.0);
//     ty = NetworkTableInstance.getDefault().getTable("limelight-new").getEntry("ty").getDouble(0.0);

//     double[] temp = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };// Defult getEntry
//     result = NetworkTableInstance.getDefault().getTable("limelight-new").getEntry("botpose_wpiblue").getDoubleArray(temp);
//     SmartDashboard.putNumber("tv", tv);
//     SmartDashboard.putNumber("tid", tid);
//     SmartDashboard.putNumber("tx", tx);
//     SmartDashboard.putNumber("ty", ty);
//     // This method will be called once per scheduler run

//     translation3d = new Translation3d(result[0], result[1], result[2]);

//     SmartDashboard.putString("translation", translation3d.toString());
//     rotation3d = new Rotation3d(Units.degreesToRadians(result[3]), Units.degreesToRadians(result[4]),
//     Units.degreesToRadians(result[5]));
//     SmartDashboard.putString("rotation", rotation3d.toString());

//     Pose3d pose3d = new Pose3d(translation3d, rotation3d);
//     botPose2d = pose3d.toPose2d();
//     SmartDashboard.putString("BotPose2d", botPose2d.toString());
//   }

//   public void ledOff() {
//     table.getEntry("ledMode").setNumber(1);
//   }

//   public void ledOn() {
//     table.getEntry("ledMode").setNumber(3);
//   }

//   public void ledToggle() {
//     if(ledStatus==1){
//       table.getEntry("ledMode").setNumber(3);
//       ledStatus = 3;
//     }
//     else{
//       table.getEntry("ledMode").setNumber(1);
//       ledStatus = 1;      
//     }
//   }
// }
