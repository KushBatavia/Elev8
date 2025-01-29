// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// import com.revrobotics.SparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
  // CHANGE ALL CAN IDS
  public SparkMax armIntakeMotor = new SparkMax(10, MotorType.kBrushless);
  public TalonFX leftBaseMotor = new TalonFX(21, "rio"); // Left Motor is the follower
  public TalonFX rightBaseMotor = new TalonFX(22, "rio"); // Right Motor is the master
  // public Follower leftBaseMotorFollower = new Follower(BASE_CAN_ID, true);
  public TalonFX middleMotor = new TalonFX(23, "rio");
  public CANcoder canCoderLeft = new CANcoder(24, "rio");
  // public CANcoder canCoderRight = new CANcoder(2, "rio");
  public CANcoder canCoderMiddle = new CANcoder(25, "rio");
  public DigitalInput beamBreaker1 = new DigitalInput(1);
  public DigitalInput beamBreaker2 = new DigitalInput(2);
  SparkMaxConfig config = new SparkMaxConfig();

  public MotionMagicDutyCycle mMotionMagicDutyCycleBase;
  public MotionMagicDutyCycle mMotionMagicDutyCycleMiddle;

  public static final double MIDDLE_UPPER_LIMIT = 182;
  public static final double MIDDLE_DOWNER_LIMIT = 244;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    leftBaseMotor.setControl(new Follower(rightBaseMotor.getDeviceID(), true));
    // Arm Intake Motor Config

    config
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    config.encoder
        .positionConversionFactor(1000)
        .velocityConversionFactor(1000);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1.0, 0.0, 0.0);

    armIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // // Left Base Motor Config
    // leftBaseMotor.getConfigurator().apply(new TalonFXConfiguration());
    // leftBaseMotor.setInverted(false);

    // TalonFXConfiguration leftBaseMotorTalonConfigs = new TalonFXConfiguration();

    // leftBaseMotorTalonConfigs.MotorOutput.withPeakForwardDutyCycle(1);
    // leftBaseMotorTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-1);
    // leftBaseMotorTalonConfigs.Slot0.withKP(0.2);
    // leftBaseMotorTalonConfigs.Slot0.withKG(0);
    // leftBaseMotorTalonConfigs.Slot0.withKI(0);
    // leftBaseMotorTalonConfigs.Slot0.withKD(0);

    // leftBaseMotorTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    // leftBaseMotorTalonConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    // leftBaseMotorTalonConfigs.MotionMagic.withMotionMagicAcceleration(1200);
    // leftBaseMotorTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(1200);

    // mMotionMagicDutyCycleBase = new MotionMagicDutyCycle(0);
    // PositionDutyCycle leftBaseMotorPositionDutyCycle = new PositionDutyCycle(0);
    // leftBaseMotor.getConfigurator().apply(leftBaseMotorTalonConfigs, 0.050);
    // leftBaseMotor.setPosition(canCoderLeft.getAbsolutePosition().getValueAsDouble()
    // * Constants.arm_gear_ratio);

    // Right Base Motor Config
    rightBaseMotor.getConfigurator().apply(new TalonFXConfiguration());
    rightBaseMotor.setInverted(false);

    TalonFXConfiguration rightBaseMotorTalonConfigs = new TalonFXConfiguration();

    rightBaseMotorTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.05);
    rightBaseMotorTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.05);
    rightBaseMotorTalonConfigs.Slot0.withKP(0.2);
    // rightBaseMotorTalonConfigs.Slot0.withKG(0);
    rightBaseMotorTalonConfigs.Slot0.withKI(0);
    rightBaseMotorTalonConfigs.Slot0.withKD(0);

    rightBaseMotorTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    rightBaseMotorTalonConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    rightBaseMotorTalonConfigs.MotionMagic.withMotionMagicAcceleration(1200);
    rightBaseMotorTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(1200);

    mMotionMagicDutyCycleBase = new MotionMagicDutyCycle(0);
    PositionDutyCycle rightBaseMotorPositionDutyCycle = new PositionDutyCycle(0);
    rightBaseMotor.getConfigurator().apply(rightBaseMotorTalonConfigs, 0.050);
    rightBaseMotor.setPosition(canCoderLeft.getAbsolutePosition().getValueAsDouble() * Constants.arm_base_gear_ratio);

    // Middle Motor Config
    middleMotor.getConfigurator().apply(new TalonFXConfiguration());
    middleMotor.setInverted(false);

    TalonFXConfiguration middleMotorTalonConfigs = new TalonFXConfiguration();

    middleMotorTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.05);
    middleMotorTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.05);
    middleMotorTalonConfigs.Slot0.withKP(0.2);
    // middleMotorTalonConfigs.Slot0.withKG(0);
    middleMotorTalonConfigs.Slot0.withKI(0);
    middleMotorTalonConfigs.Slot0.withKD(0);

    middleMotorTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    middleMotorTalonConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    middleMotorTalonConfigs.MotionMagic.withMotionMagicAcceleration(1200);
    middleMotorTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(1200);

    mMotionMagicDutyCycleMiddle = new MotionMagicDutyCycle(0);
    PositionDutyCycle middleMotorPositionDutyCycle = new PositionDutyCycle(0);
    middleMotor.getConfigurator().apply(middleMotorTalonConfigs, 0.050);
    middleMotor.setPosition(canCoderMiddle.getAbsolutePosition().getValueAsDouble() * Constants.arm_middle_gear_ratio);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Left Base Motor Reading", (getLeftBasePos()));
    // SmartDashboard.putNumber("Left Base CANCoder", (getLeftBaseCANPos()) * 360);

    SmartDashboard.putNumber("Right Base Motor Reading", (getRightBasePos()));
    SmartDashboard.putNumber("Right Base CANCoder", (getRightBaseCANPos()) * 360);

    SmartDashboard.putNumber("Middle Motor Reading", (getMiddlePos()));
    SmartDashboard.putNumber("Middle CANCoder", (getMiddleCANPos()) * 360);

    armIntakeMotor.setVoltage(3);

    // This method will be called once per scheduler run
  }

  public void setArmIntakeMotor(double voltage) {
    armIntakeMotor.set(voltage);
  }

  public boolean getBeam1() {
    return beamBreaker1.get();
  }

  public boolean getBeam2() {
    return beamBreaker2.get();
  }

  // public double getLeftBaseCANPos() {
  // double position = canCoderLeft.getAbsolutePosition().getValueAsDouble();
  // return position;
  // }

  // public void setLeftBasePos(double angle) {
  // leftBaseMotor.setControl(mMotionMagicDutyCycleBase.withPosition((angle *
  // Constants.arm_gear_ratio) / 360).withSlot(0));
  // }

  // public double getLeftBasePos() {
  // return (leftBaseMotor.getPosition().getValueAsDouble() /
  // Constants.arm_gear_ratio) * 360;
  // }

  public double getRightBaseCANPos() {
    double position = canCoderLeft.getAbsolutePosition().getValueAsDouble();
    return position;
  }

  public void setRightBasePos(double angle) {
    rightBaseMotor
        .setControl(mMotionMagicDutyCycleBase.withPosition((angle * Constants.arm_base_gear_ratio) / 360).withSlot(0));
  }

  public double getRightBasePos() {
    return (rightBaseMotor.getPosition().getValueAsDouble() / Constants.arm_base_gear_ratio) * 360;
  }

  public double getMiddleCANPos() {
    double position = canCoderMiddle.getAbsolutePosition().getValueAsDouble();
    return position;
  }

  public void setMiddlePos(double angle) {
    angle = Math.max(angle, MIDDLE_UPPER_LIMIT);
    angle = Math.min(angle, MIDDLE_DOWNER_LIMIT);
    middleMotor.setControl(
        mMotionMagicDutyCycleMiddle.withPosition((angle * Constants.arm_middle_gear_ratio) / 360).withSlot(0));
  }

  public double getMiddlePos() {
    return (middleMotor.getPosition().getValueAsDouble() / Constants.arm_middle_gear_ratio) * 360;
  }

  public void resetMiddle() {
    middleMotor.setPosition(getMiddlePos() * Constants.arm_middle_gear_ratio);
  }

  public void resetBase() {
    rightBaseMotor.setPosition(getRightBasePos() * Constants.arm_base_gear_ratio);
  }
}
