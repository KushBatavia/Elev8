// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class SparkMaxSubsystem extends SubsystemBase {
//   public SparkMax armIntakeMotor = new SparkMax(10, MotorType.kBrushless);
//   SparkMaxConfig config = new SparkMaxConfig();
//   /** Creates a new SparkMaxSubsystem. */
//   public SparkMaxSubsystem() {
//     config
//         .inverted(true)
//         .idleMode(IdleMode.kBrake);
//     config.encoder
//         .positionConversionFactor(1000)
//         .velocityConversionFactor(1000);
//     config.closedLoop
//         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//         .pid(1.0, 0.0, 0.0);

//     armIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//   }
//   public void setArmIntakeMotor(double voltage) {
//     armIntakeMotor.set(voltage);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     armIntakeMotor.setVoltage(0);
//   }
// }
