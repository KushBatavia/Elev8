// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GroundIntakeSubsystem extends SubsystemBase {
  public TalonFX intakeMotor = new TalonFX(14, "rio");
  public TalonFX hoodIntakeMotor = new TalonFX(15, "rio");
  public static DigitalInput beamBreaker1 = new DigitalInput(0);
  public CANcoder canCoder = new CANcoder(10, "rio"); 
  public MotionMagicDutyCycle mMotionMagicDutyCycle;

  private final double UPPER_LIMIT = 0;
  private final double DOWNER_LIMIT = 0;

  public static double intakeState = 1;
  public static boolean coralState = false;

  
  /** Creates a new IntakeSubsystem. */
  @SuppressWarnings("removal")
  public GroundIntakeSubsystem() {
    // Intake Motor Config
    intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    intakeMotor.setInverted(false);

    TalonFXConfiguration intakeMotorTalonConfigs  = new TalonFXConfiguration();

    intakeMotorTalonConfigs.MotorOutput.withPeakForwardDutyCycle(1);
    intakeMotorTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-1);
    intakeMotorTalonConfigs.Slot0.withKP(0.2);
    intakeMotorTalonConfigs.Slot0.withKG(0);
    intakeMotorTalonConfigs.Slot0.withKI(0);
    intakeMotorTalonConfigs.Slot0.withKD(0);

    intakeMotorTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    intakeMotorTalonConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    // motor1TalonConfigs.MotionMagic.withMotionMagicAcceleration(1200);
    // motor1TalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(1200);

    // MotionMagicDutyCycle mMotionMagicDutyCycle = new MotionMagicDutyCycle(0);
    // PositionDutyCycle intakeMotorPositionDutyCycle = new PositionDutyCycle(0);
    intakeMotor.getConfigurator().apply(intakeMotorTalonConfigs, 0.050);
    intakeMotor.setPosition(0);

    // Hood Intake Motor Config
    hoodIntakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    hoodIntakeMotor.setInverted(false);

    TalonFXConfiguration hoodIntakeMotorTalonConfigs  = new TalonFXConfiguration();

    hoodIntakeMotorTalonConfigs.MotorOutput.withPeakForwardDutyCycle(1);
    hoodIntakeMotorTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-1);
    hoodIntakeMotorTalonConfigs.Slot0.withKP(0.2);
    hoodIntakeMotorTalonConfigs.Slot0.withKG(0);
    hoodIntakeMotorTalonConfigs.Slot0.withKI(0);
    hoodIntakeMotorTalonConfigs.Slot0.withKD(0);

    hoodIntakeMotorTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    hoodIntakeMotorTalonConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    hoodIntakeMotorTalonConfigs.MotionMagic.withMotionMagicAcceleration(1200);
    hoodIntakeMotorTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(1200);

    mMotionMagicDutyCycle = new MotionMagicDutyCycle(0);
    PositionDutyCycle hoodIntakeMotorPositionDutyCycle = new PositionDutyCycle(0);
    hoodIntakeMotor.getConfigurator().apply(hoodIntakeMotorTalonConfigs, 0.050);
    hoodIntakeMotor.setPosition(canCoder.getAbsolutePosition().getValueAsDouble() * Constants.hood_gear_ratio);
  }

  public boolean getBeamBreaker1() {
    return beamBreaker1.get();
  }

  public void setIntakeMotor(double voltage) {
    intakeMotor.set(voltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Motor Reading", (getHoodPos()));
    SmartDashboard.putNumber("Hood CANCoder", (getPos()));
    // This method will be called once per scheduler run
  }

  public double getPos() {
    double position = canCoder.getAbsolutePosition().getValueAsDouble()*360;
    return position;
  }

  public void setPos(double angle) {
    angle = Math.max(angle, UPPER_LIMIT);
    angle = Math.min(angle, DOWNER_LIMIT);
    hoodIntakeMotor.setControl(mMotionMagicDutyCycle.withPosition((angle * Constants.hood_gear_ratio) / 360).withSlot(0));
  }

  public double getHoodPos() {
    return (hoodIntakeMotor.getPosition().getValueAsDouble() / Constants.hood_gear_ratio) * 360;
  }

  public double getCurrent() {
    return intakeMotor.getStatorCurrent().getValueAsDouble();
  }
}