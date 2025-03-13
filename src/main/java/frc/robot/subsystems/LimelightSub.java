// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class LimelightSub extends SubsystemBase {
  public CommandSwerveDrivetrain m_swerve;
  private Matrix<N3, N1> visionStdDevs;
  public LimelightSub(){
      }

  public void updatePose() {
    if (!LimelightHelpers.getTV("limelight-new")) return; // Check if target is visible
    Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-new");
    double timestamp = Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture("limelight-new") / 1000.0;
    m_swerve.addVisionMeasurement(visionPose, timestamp, visionStdDevs);
  }

  @Override
  public void periodic() {
    visionStdDevs.set(0, 0, 0.5); // x uncertainty
    visionStdDevs.set(1, 0, 0.5); // y uncertainty
    visionStdDevs.set(2, 0, Math.toRadians(5)); // theta uncertainty
    updatePose();
  }
}
