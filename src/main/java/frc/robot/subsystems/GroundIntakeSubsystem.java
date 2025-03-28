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
  public TalonFX intakeBottomMotor = new TalonFX(14, "rio");
  public TalonFX intakeTopMotor = new TalonFX(14, "rio");
  public TalonFX hoodMotor = new TalonFX(15, "rio");
  public CANcoder canCoder = new CANcoder(10, "rio"); 
  public MotionMagicDutyCycle mMotionMagicDutyCycle;

  private final double SET_VALUE_TEMP = 0;
  private final double UPPER_LIMIT = SET_VALUE_TEMP;
  private final double DOWNER_LIMIT = SET_VALUE_TEMP;

  public static double intakeState = 1;
  public static boolean coralState = false;

  public static DigitalInput bottomBeam = new DigitalInput(1);
  public static DigitalInput topBeam = new DigitalInput(0);

  
  /** Creates a new IntakeSubsystem. */
  @SuppressWarnings("removal")
  public GroundIntakeSubsystem() {

    //Intake Top MOtor COnfig
    intakeTopMotor.getConfigurator().apply(new TalonFXConfiguration());
    intakeTopMotor.setInverted(false);

    TalonFXConfiguration intakeTopMotorTalonConfigs  = new TalonFXConfiguration();

    intakeTopMotorTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.05);
    intakeTopMotorTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.05);
    intakeTopMotorTalonConfigs.Slot0.withKP(0.2);
    intakeTopMotorTalonConfigs.Slot0.withKG(0);
    intakeTopMotorTalonConfigs.Slot0.withKI(0);
    intakeTopMotorTalonConfigs.Slot0.withKD(0);

    intakeTopMotorTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    intakeTopMotorTalonConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    // motor1TalonConfigs.MotionMagic.withMotionMagicAcceleration(1200);
    // motor1TalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(1200);

    // MotionMagicDutyCycle mMotionMagicDutyCycle = new MotionMagicDutyCycle(0);
    // PositionDutyCycle intakeMotorPositionDutyCycle = new PositionDutyCycle(0);
    intakeTopMotor.getConfigurator().apply(intakeTopMotorTalonConfigs, 0.050);
    intakeTopMotor.setPosition(0);


    // Intake Bottom Motor Config
    intakeBottomMotor.getConfigurator().apply(new TalonFXConfiguration());
    intakeBottomMotor.setInverted(false);

    TalonFXConfiguration intakeBottomMotorTalonConfigs  = new TalonFXConfiguration();

    intakeBottomMotorTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.05);
    intakeBottomMotorTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.05);
    intakeBottomMotorTalonConfigs.Slot0.withKP(0.2);
    intakeBottomMotorTalonConfigs.Slot0.withKG(0);
    intakeBottomMotorTalonConfigs.Slot0.withKI(0);
    intakeBottomMotorTalonConfigs.Slot0.withKD(0);

    intakeBottomMotorTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    intakeBottomMotorTalonConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    // motor1TalonConfigs.MotionMagic.withMotionMagicAcceleration(1200);
    // motor1TalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(1200);

    // MotionMagicDutyCycle mMotionMagicDutyCycle = new MotionMagicDutyCycle(0);
    // PositionDutyCycle intakeMotorPositionDutyCycle = new PositionDutyCycle(0);
    intakeBottomMotor.getConfigurator().apply(intakeBottomMotorTalonConfigs, 0.050);
    intakeBottomMotor.setPosition(0);

    // Hood Intake Motor Config
    hoodMotor.getConfigurator().apply(new TalonFXConfiguration());
    hoodMotor.setInverted(false);

    TalonFXConfiguration hoodMotorTalonConfigs  = new TalonFXConfiguration();

    hoodMotorTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.05);
    hoodMotorTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.05);
    hoodMotorTalonConfigs.Slot0.withKP(0.2);
    hoodMotorTalonConfigs.Slot0.withKG(0);
    hoodMotorTalonConfigs.Slot0.withKI(0);
    hoodMotorTalonConfigs.Slot0.withKD(0);

    hoodMotorTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    hoodMotorTalonConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    hoodMotorTalonConfigs.MotionMagic.withMotionMagicAcceleration(1200);
    hoodMotorTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(1200);

    mMotionMagicDutyCycle = new MotionMagicDutyCycle(0);
    PositionDutyCycle hoodIntakeMotorPositionDutyCycle = new PositionDutyCycle(0);
    hoodMotor.getConfigurator().apply(hoodMotorTalonConfigs, 0.050);
    hoodMotor.setPosition(canCoder.getAbsolutePosition().getValueAsDouble() * Constants.hood_gear_ratio);
  }

  public void setBottomMotor(double voltage) {
    voltage = Math.max(voltage, 0);
    voltage = Math.min(voltage, 1);
    intakeBottomMotor.set(voltage*16);
  }
  public void setTopMotor(double voltage) {
    intakeTopMotor.set(voltage*16);
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
    hoodMotor.setControl(mMotionMagicDutyCycle.withPosition((angle * Constants.hood_gear_ratio) / 360).withSlot(0));
  }

  public double getHoodPos() {
    return (hoodMotor.getPosition().getValueAsDouble() / Constants.hood_gear_ratio) * 360;
  }

  public boolean getBottomBeam() {//gets intake beam
    return bottomBeam.get();
  }

  public boolean getTopBeam() {//gets outtake beam
    return topBeam.get();
  }
}