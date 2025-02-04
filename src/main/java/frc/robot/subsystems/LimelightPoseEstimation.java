// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightPoseEstimation extends SubsystemBase {
  private final CommandSwerveDrivetrain m_swerve;
  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-new");
  static NetworkTableEntry valueOfPoses = table.getEntry("botpose_wpiblue");

  // FOR POSE 2D ESTIMATION ADD VALUES LATE 
  private final double X_VALUE = 3; 
  private final double Y_VALUE = 2; 

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors.
  // Smaller numbers will cause the filter to "trust" the estimate from that
  // particular component more than the others.
  // This in turn means the particualr component will have a stronger influence on
  // the final pose estimate.
  private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2));
  private static SwerveDrivePoseEstimator poseEstimator;
  public int pipelineindex = 0;
  public int[] Ids ={1, 2, 3, 4, 5, 6, 7, 8, 14, 15, 16, 17, 18, 11, 12, 13, 10, 9}; // ADD MORE IDS 
  public double tv, tid, x, y, angle, prev_angle;
  Pose2d inlineBotPose = new Pose2d(X_VALUE, Y_VALUE, new Rotation2d(0));
  
  /** Creates a new LimelightPoseEstimation. */
  private final Field2d field2d = new Field2d();
  public static Pose2d botP = new Pose2d();  

  public double[] network_tables = NetworkTableInstance.getDefault().getTable("limelight-new").getEntry("botpose").getDoubleArray(new double[6]);


  public LimelightPoseEstimation(CommandSwerveDrivetrain m_swerve, Pose2d pose) {
    this.m_swerve = m_swerve; 
    /*
    initialPoseMeters - The starting pose estimate.
    stateStdDevs - Standard deviations of the pose estimate (x position in meters, y position in meters, and heading in radians). 
    Increase these numbers to trust your state estimate less.
    visionMeasurementStdDevs - Standard deviations of the vision pose measurement (x position in meters, y position in meters, and heading in radians). 
    Increase these numbers to trust the vision pose measurement less.
    */

    // poseEstimator = new SwerveDrivePoseEstimator(
    //   Constants.Swerve.swerveKinematics,
    //     // m_swerve.getGyroYaw(),
    //     // m_swerve.getModulePositions(),
    //     pose,
    //     stateStdDevs,
    //     visionMeasurementStdDevs);
    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void periodic() {
    // Update pose estimator with visible targets
    // latest pipeline result
    double[] temp = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };// Defult getEntry
    double[] result = valueOfPoses.getDoubleArray(temp);
    double timestamp = Timer.getFPGATimestamp();

    // This method will be called once per scheduler run
  }

  public static Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
  
  public double getPoseX() {
    return poseEstimator.getEstimatedPosition().getX();
  }
  
  public double getPoseY() {
    return poseEstimator.getEstimatedPosition().getY();
  }

  public double getPoseTheta() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }
  
  public Rotation2d getPoseRotatation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  public void setPose(Pose2d pose) {
    // poseEstimator.resetPosition(m_swerve.getGyro(), m_swerve.getModulePositions(), pose);
  }

  public void IndexUpdate(int pipeline) {
    pipelineindex = pipeline;
  }

  public void setTrajectoryField2d(Trajectory trajectory) {
    field2d.getObject("traj").setTrajectory(trajectory);
  }
}
