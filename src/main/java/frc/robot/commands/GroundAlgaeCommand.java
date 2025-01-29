// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundAlgaeCommand extends Command {
  private final GroundIntakeSubsystem m_intake;
  private final double CURRENT_THRESHOLD = 0.5;
  private boolean returnflag;
  /** Creates a new IntakeCommand. */
  
  public GroundAlgaeCommand(GroundIntakeSubsystem m_intake) {
    this.m_intake = m_intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_intake.setIntakeMotor(1.5);
    // m_intake.setPos(0);
    returnflag = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getCurrent() > CURRENT_THRESHOLD) {
      // m_intake.setIntakeMotor(0); set intake motor to 0 stop intaking
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
