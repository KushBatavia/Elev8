// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new Grabber. */
  private final static TalonFX grabber = new TalonFX(20);
  public static DigitalInput beamBreaker1 = new DigitalInput(0);
  private final static double waitTime = 1;
    static double initTime;
    static double currentTime;
      
      public GrabberSubsystem() {
    
        grabber.getConfigurator().apply(new TalonFXConfiguration());
              grabber.setInverted(false);
          
              TalonFXConfiguration grabberTalonConfigs = new TalonFXConfiguration();
          
              grabberTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.1);
              grabberTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.1);
              grabberTalonConfigs.Slot0.withKP(1.1);
              grabberTalonConfigs.Slot0.withKG(0.08); 
              grabberTalonConfigs.Slot0.withKI(0);
              grabberTalonConfigs.Slot0.withKD(0);
          
              grabberTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
              grabberTalonConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
          
              grabberTalonConfigs.MotionMagic.withMotionMagicAcceleration(325);
              grabberTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(650);
          
              // PositionDutyCycle grabberPositionDutyCycle = new PositionDutyCycle(0);
    
              grabber.getConfigurator().apply(grabberTalonConfigs, 0.050);
      }
    
  //     public static void Grab()
  //     {
  //     initTime = Timer.getFPGATimestamp();
  //     while(!beamBreaker1.get())
  //     {
  //       currentTime = Timer.getFPGATimestamp();
  //       grabber.set(0.3);
  //       if (Math.abs(currentTime - initTime)>=waitTime)
  //   }
  //   grabber.set(0);
  // }

  public static boolean GetBeam()
  {
    return beamBreaker1.get();
  }

  public static void RunGrabber(double power)
  {
    grabber.set(power);
  }

  // public static void Leave()
  // {
  //   while(beamBreaker1.get())
  //   {
  //     grabber.set(-0.3);
  //   }
  //   grabber.set(0);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
