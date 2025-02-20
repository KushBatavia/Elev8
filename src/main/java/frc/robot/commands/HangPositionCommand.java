// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HangPositionCommand extends Command {
  /** Creates a new HangPositionCommand. */
  private ArmSubsystem m_arm = new ArmSubsystem();
  private GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem();
  private double state;
  private double armMiddlePos;
  private double armBasePos;
  private double SET_ANGLE_Temp;
  private double SET_POWER_Temp;
  private boolean returnFlag;
  public HangPositionCommand(ArmSubsystem m_arm, GroundIntakeSubsystem m_ground) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_ground = m_ground;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    m_arm.setMiddlePos(393+40);
    m_arm.setRightBasePos(193);
    Constants.killFlag = false;
    returnFlag = false;    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.getRightBasePos() < 230 && state == 0){
    // SET ARM INTAKE MOTOR FOR SMTH IDK
    //Figure out which motors are being used
      state = 1;
    }
    if(state == 1){
      m_ground.setPos(100);
      state = 2; 
      returnFlag = true;
    }
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
