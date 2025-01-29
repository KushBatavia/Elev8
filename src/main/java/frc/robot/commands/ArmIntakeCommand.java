// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmIntakeCommand extends Command {
  private final ArmSubsystem m_arm; 
  /** Creates a new ArmCommand. */
  public ArmIntakeCommand(ArmSubsystem m_arm) {
    this.m_arm = m_arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_arm.setArmIntakeMotor(1.5);
    // m_arm.setBasePos(0);
    // m_arm.setMiddlePos(0);
    // m_arm.setArmIntakeMotor(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_arm.getBeam1() && m_arm.getBeam2()) {
      // m_arm.setArmIntakeMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
