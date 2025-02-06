// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeUpRemovalCommand extends Command {
  private ArmSubsystem m_arm = new ArmSubsystem();
  private double SET_ANGLE_Temp;
  private boolean returnFlag;
  /** Creates a new AlgaeUpRemovalCommand. */
  public AlgaeUpRemovalCommand(ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setMiddlePos(SET_ANGLE_Temp);
    m_arm.setRightBasePos(SET_ANGLE_Temp);
    ArmSubsystem.algaeFlag = true;    
    ArmSubsystem.armState = 3;
    
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
    return returnFlag;
  }
}
