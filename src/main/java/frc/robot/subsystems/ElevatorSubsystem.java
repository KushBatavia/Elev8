// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX leftElevator = new TalonFX(17);
  private final static TalonFX rightElevator = new TalonFX(18);
  private static double desiredPosition;
  
    private final static MotionMagicDutyCycle mMagic = new MotionMagicDutyCycle(0); 
        /** Creates a new ExampleSubsystem. */
        public ElevatorSubsystem() {
          leftElevator.setControl(new Follower(rightElevator.getDeviceID(), true));
      
          // Right Base Motor Config
          rightElevator.getConfigurator().apply(new TalonFXConfiguration());
          rightElevator.setInverted(false);
      
          TalonFXConfiguration rightElevatorTalonConfigs = new TalonFXConfiguration();
      
          rightElevatorTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.1);
          rightElevatorTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.1);
          rightElevatorTalonConfigs.Slot0.withKP(1.1);
          rightElevatorTalonConfigs.Slot0.withKG(0.12); 
          rightElevatorTalonConfigs.Slot0.withKI(0);
          rightElevatorTalonConfigs.Slot0.withKD(0);
      
          rightElevatorTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
          rightElevatorTalonConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
      
          rightElevatorTalonConfigs.MotionMagic.withMotionMagicAcceleration(450);
          rightElevatorTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(900);
      
          // PositionDutyCycle rightElevatorPositionDutyCycle = new PositionDutyCycle(0);

          rightElevator.getConfigurator().apply(rightElevatorTalonConfigs, 0.050);
          rightElevator.setPosition(0);
        }
      
        /**
         * Example command factory method.
         *
         * @return a command
         */
        public Command exampleMethodCommand() {
          // Inline construction of command goes here.
          // Subsystem::RunOnce implicitly requires `this` subsystem.
          return runOnce(
              () -> {
                /* one-time action goes here */
              });
        }
      
        /**
         * An example method querying a boolean state of the subsystem (for example, a digital sensor).
         *
         * @return value of some boolean subsystem state, such as a digital sensor.
         */
        public boolean exampleCondition() {
          // Query some boolean state, such as a digital sensor.
          return false;
        }
        public static boolean ReachedPosition()
        {
          return (Math.abs(desiredPosition-rightElevator.getPosition().getValueAsDouble())>=20); //Not sure if this is right
        }
        public static void DropPos()
        {
          desiredPosition = 0;
         rightElevator.setControl(mMagic.withPosition((0 * Constants.elevator_gear_ratio) / 360).withSlot(0));
        }
        public static void L1()
        {
          desiredPosition = 1000;
         rightElevator.setControl(mMagic.withPosition((1000 * Constants.elevator_gear_ratio) / 360).withSlot(0));
        }
      
        public static void L2()
        {
          desiredPosition = 1500;
         rightElevator.setControl(mMagic.withPosition((1500 * Constants.elevator_gear_ratio) / 360).withSlot(0));
  }

  public static void L3()
  {
    desiredPosition = 2000;
   rightElevator.setControl(mMagic.withPosition((2000 * Constants.elevator_gear_ratio) / 360).withSlot(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("ReachedPosition", ReachedPosition());
    SmartDashboard.putNumber("desiredPos", desiredPosition);
    SmartDashboard.putNumber("currentPos", rightElevator.getPosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
