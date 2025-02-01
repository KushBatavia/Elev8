// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HangPositionCommand extends Command {
  /** Creates a new HangPositionCommand. */
  private ArmSubsystem m_arm = new ArmSubsystem();
  private GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem();
  private double state = 0;
  private double armMiddlePos;
  private double armBasePos;
  private double SET_ANGLE_Temp;
  private double SET_POWER_Temp;
  public HangPositionCommand(ArmSubsystem m_arm, GroundIntakeSubsystem m_ground) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_ground = m_ground;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_ground.getPos()>255 || m_ground.getPos()<200) {
      state = 0.5;
    }
    if(state == 0.5){
      //Figure out which motors are being used
    }
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
