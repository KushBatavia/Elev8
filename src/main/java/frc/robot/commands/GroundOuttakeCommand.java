// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundOuttakeCommand extends Command {
  private final GroundIntakeSubsystem m_intake;
  private boolean returnflag;
  /** Creates a new IntakeCommand. */
  
  public GroundOuttakeCommand(GroundIntakeSubsystem m_intake) {
    this.m_intake = m_intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getBeamBreaker1()) {
      // m_intake.setIntakeMotor(reverse for coral);
      // m_intake.setPos(angle for coral);
      // returnflag = true;
    } else if (!m_intake.getBeamBreaker1()) {
      // m_intake.setIntakeMotor(reverse for algae);
      // m_intake.setPos(another angle for algae);
      // returnflag = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return returnflag;
  }
}
