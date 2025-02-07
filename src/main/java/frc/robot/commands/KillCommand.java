// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.SparkMaxSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class KillCommand extends Command {
  private ArmSubsystem m_arm;
  private GroundIntakeSubsystem m_ground;
  private SparkMaxSubsystem m_spark;
  private RobotContainer container;
  /** Creates a new KillCommand. */
  public KillCommand(ArmSubsystem m_arm, GroundIntakeSubsystem m_ground, SparkMaxSubsystem m_spark) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_ground = m_ground;
    this.m_spark = m_spark;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.killFlag = true;
    m_spark.setArmIntakeMotor(0);
    m_arm.setHangarMotorPower(0);
    m_arm.middleMotor.set(0);
    m_arm.rightBaseMotor.set(0);
    m_ground.setIntakeMotor(0);
    m_ground.hoodIntakeMotor.set(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
