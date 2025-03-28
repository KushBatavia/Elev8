// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L3Command extends Command {
  private ArmSubsystem m_arm;
  private GroundIntakeSubsystem m_ground; 
  private double state = 0;
  private double armMiddlePos;
  private double armBasePos;
  private double SET_ANGLE_Temp;
  private boolean returnFlag;
  /** Creates a new L3Command. */
  public L3Command(ArmSubsystem m_arm, GroundIntakeSubsystem m_ground) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_ground = m_ground;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.killFlag = false;
    ArmSubsystem.algaeFlag = false;
    returnFlag = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ArmSubsystem.armState == 1){
      if(state == 0.5){
        m_arm.setRightBasePos(217);
        state = 1;
      }
      if(state == 1 && Math.abs(m_arm.getMiddleCANPos() - 217)<3){
        m_arm.setRightBasePos(393);
        m_arm.setMiddlePos(187);
        state = 3;
      }
      if(state == 3) {    
        ArmSubsystem.shootFlag = true;
        ArmSubsystem.armState = 1;
        returnFlag = true;
        state = 4;
      }    
   }else if(ArmSubsystem.armState == 2){
      returnFlag = true;
   }else{
      m_arm.setMiddlePos(393);
      m_arm.setRightBasePos(187);
      state = 1;
      if(state == 1) {    
        ArmSubsystem.shootFlag = true;
        ArmSubsystem.armState = 1;
        returnFlag = true;
      } 
    }
    if(armMiddlePos<SET_ANGLE_Temp  && armBasePos<SET_ANGLE_Temp && state == 1) {
      ArmSubsystem.shootFlag = true;
      ArmSubsystem.armState = 2; 
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
