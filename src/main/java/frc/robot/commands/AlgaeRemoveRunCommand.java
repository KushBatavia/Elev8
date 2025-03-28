// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SparkMaxSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeRemoveRunCommand extends Command {
  private SparkMaxSubsystem m_spark ;
  private double state;
  private double armMiddlePos;
  private double armBasePos;
  private double SET_ANGLE_Temp; //I dont have values, so these are just temporary variable created that im using everywhere.
  private double SET_POWER_Temp;//Not the same everywhere, just follow the logic dont look at the variable being used
  private double prevT;
  private double lastT;
  private boolean returnFlag;
  /** Creates a new AlgaeRemoveRunCommand. */
  public AlgaeRemoveRunCommand(SparkMaxSubsystem m_spark) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_spark = m_spark;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.killFlag = false;
    state = 0;
    returnFlag = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    prevT = Timer.getFPGATimestamp();
    if(armMiddlePos>SET_ANGLE_Temp  && armBasePos>SET_ANGLE_Temp && ArmSubsystem.algaeFlag) {
      m_spark.setArmIntakeMotor(-0.3);
      lastT = Timer.getFPGATimestamp();
      state = 1;
    }
    if(state == 1 && Math.abs(prevT - lastT) > 0.5) {
        m_spark.setArmIntakeMotor(0);
        state = 2;
      }
      returnFlag = true;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return returnFlag||Constants.killFlag;
  }
}
