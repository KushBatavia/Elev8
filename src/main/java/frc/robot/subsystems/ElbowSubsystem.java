// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElbowSubsystem extends SubsystemBase {
  /** Creates a new ElbowSubsystem. */
  private final static TalonFX elbow = new TalonFX(19);
  private final static MotionMagicDutyCycle mMagic = new MotionMagicDutyCycle(0); 


  public ElbowSubsystem() {
    elbow.getConfigurator().apply(new TalonFXConfiguration());
          elbow.setInverted(false);
      
          TalonFXConfiguration elbowTalonConfigs = new TalonFXConfiguration();
      
          elbowTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.1);
          elbowTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.1);
          elbowTalonConfigs.Slot0.withKP(0.3);
          elbowTalonConfigs.Slot0.withKG(0.03); 
          elbowTalonConfigs.Slot0.withKI(0);
          elbowTalonConfigs.Slot0.withKD(0);
      
          elbowTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
          elbowTalonConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
      
          elbowTalonConfigs.MotionMagic.withMotionMagicAcceleration(75);
          elbowTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(150);
      
          // PositionDutyCycle elbowPositionDutyCycle = new PositionDutyCycle(0);

          elbow.getConfigurator().apply(elbowTalonConfigs, 0.050);
          elbow.setPosition(0);
  }


  public static void SetDrop()
  {
    elbow.setControl(mMagic.withPosition((-130 * Constants.elbow_gear_ratio) / 360).withSlot(0));
  }

  public static void SetPick()
  {
    elbow.setControl(mMagic.withPosition((-7.5 * Constants.elbow_gear_ratio) / 360).withSlot(0));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
