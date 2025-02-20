// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
  public TalonFX leftBaseMotor = new TalonFX(21, "rio"); // Left Motor is the follower
  public TalonFX rightBaseMotor = new TalonFX(22, "rio"); // Right Motor is the master
  public TalonFX middleMotor = new TalonFX(23, "rio");
  public TalonFX hangarMotor = new TalonFX(51, "rio");
  public CANcoder canCoderLeft = new CANcoder(24, "rio");
  public CANcoder canCoderMiddle = new CANcoder(25, "rio");


  public DigitalInput beamBreaker1 = new DigitalInput(1);
  public DigitalInput beamBreaker2 = new DigitalInput(2);

  public MotionMagicDutyCycle mMotionMagicDutyCycleBase;
  public MotionMagicDutyCycle mMotionMagicDutyCycleMiddle;
  public MotionMagicDutyCycle mMotionMagicDutyCycleHangar;
  

  public static final double BASE_UPPER_LIMIT = 0;
  public static final double BASE_DOWNER_LIMIT = 0;
  public static final double MIDDLE_UPPER_LIMIT = 0;
  public static final double MIDDLE_DOWNER_LIMIT = 0;

  public static boolean algaeFlag = false;

  public static boolean shootFlag = false;

  public static int armState = 0;
    //0 = Complete close
    //1 = L2
    //2 = L3
    //3 = Algae Down
    //4 = Algae Up
    //5 = Source Intake
    //6 = Hang


/*
  public static enum armStateEnum(int type){
    CLOSE, L2, L3, ALGAE_UP, ALGAEU_DOWN, SOURCE_INTAKE, HANG
  }
*/ 
  /** Creates a new ArmSubsystem. */
  // @SuppressWarnings("removal")
  public ArmSubsystem() {

    // Right Base Motor Config
    rightBaseMotor.getConfigurator().apply(new TalonFXConfiguration());
    // rightBaseMotor.setInverted(false);

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
    // PositionDutyCycle rightBaseMotorPositionDutyCycle = new PositionDutyCycle(0);
    rightBaseMotor.getConfigurator().apply(rightBaseMotorTalonConfigs, 0.050);
    rightBaseMotor.setPosition(getRightBaseCANPos()/360 * Constants.arm_base_gear_ratio);

    // Middle Motor Config
    middleMotor.getConfigurator().apply(new TalonFXConfiguration());
    // middleMotor.setInverted(false);

    TalonFXConfiguration middleMotorTalonConfigs = new TalonFXConfiguration();

    middleMotorTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.05);
    middleMotorTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.05);

    middleMotorTalonConfigs.Slot0.withKP(0.2);
    // middleMotorTalonConfigs.Slot0.withKG(0);
    middleMotorTalonConfigs.Slot0.withKI(0);
    middleMotorTalonConfigs.Slot0.withKD(0);

    middleMotorTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    middleMotorTalonConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    middleMotorTalonConfigs.MotionMagic.withMotionMagicAcceleration(80);
    middleMotorTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(150);

    mMotionMagicDutyCycleMiddle = new MotionMagicDutyCycle(0);
    // PositionDutyCycle middleMotorPositionDutyCycle = new PositionDutyCycle(0);
    middleMotor.getConfigurator().apply(middleMotorTalonConfigs, 0.050);
    middleMotor.setPosition(((getMiddleCANPos() + getRightBaseCANPos())/360) * Constants.arm_middle_gear_ratio);

    // Hangar Motor Config
    hangarMotor.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration hangarMotorTalonConfigs = new TalonFXConfiguration();
    hangarMotorTalonConfigs = new TalonFXConfiguration();

    hangarMotorTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.6);
    hangarMotorTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.6);

    hangarMotorTalonConfigs.Slot0.withKP(0.5); // .52O
    // hangarMotorTalonConfigs.Slot0.withKG(0.03);
    hangarMotorTalonConfigs.Slot0.withKI(0);
    hangarMotorTalonConfigs.Slot0.withKD(0);

    hangarMotorTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    hangarMotorTalonConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    hangarMotorTalonConfigs.MotionMagic.withMotionMagicAcceleration(80/3);
    hangarMotorTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(150/3);

    hangarMotor.getConfigurator().apply(hangarMotorTalonConfigs, 0.050);

    //left motor
    leftBaseMotor.setControl(new Follower(rightBaseMotor.getDeviceID(), true));
  }


  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Left Base Motor Reading", (getLeftBasePos()));
    // SmartDashboard.putNumber("Left Base CANCoder", (getLeftBaseCANPos()) * 360);

    SmartDashboard.putNumber("Right Base Motor Reading", (getRightBasePos()));
    SmartDashboard.putNumber("Right Base CANCoder", (getRightBaseCANPos()));

    SmartDashboard.putNumber("Middle Motor Reading", (getMiddlePos()));
    SmartDashboard.putNumber("Middle CANCoder", (getMiddleCANPos()));

    SmartDashboard.putBoolean("intake beam breaker", getBeamIntake());
    SmartDashboard.putBoolean("outtake beam breaker", getBeamOuttake());

    // This method will be called once per scheduler run
  }

  

  public boolean getBeamIntake() {//gets intake beam
    return beamBreaker1.get();
  }

  public boolean getBeamOuttake() {//gets outtake beam
    return beamBreaker2.get();
  }

  public void setHangarMotorPower(double power){
    hangarMotor.set(power);
  }

  public double getRightBaseCANPos() {
    double position = canCoderLeft.getAbsolutePosition().getValueAsDouble() * 360;
    return position;
  }

  public void setRightBasePos(double angle) {
    angle = Math.max(angle, BASE_UPPER_LIMIT);
    angle = Math.min(angle, BASE_DOWNER_LIMIT);
    rightBaseMotor.setControl(mMotionMagicDutyCycleBase.withPosition((angle * Constants.arm_base_gear_ratio) / 360).withSlot(0));
  }

  public double getRightBasePos() {
    return (rightBaseMotor.getPosition().getValueAsDouble() / Constants.arm_base_gear_ratio) * 360;
  }

  public double getMiddleCANPos() {
    double position = canCoderMiddle.getAbsolutePosition().getValueAsDouble() * 360 ;
    return position;
  }


  public void setMiddlePos(double angle) {
    angle = Math.max(angle, MIDDLE_UPPER_LIMIT);
    angle = Math.min(angle, MIDDLE_DOWNER_LIMIT);
    middleMotor.setControl(mMotionMagicDutyCycleMiddle.withPosition(((angle) * Constants.arm_middle_gear_ratio) / 360).withSlot(0));
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
