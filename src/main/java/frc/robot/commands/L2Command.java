// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L2Command extends Command {
  private ArmSubsystem m_arm = new ArmSubsystem();
  private GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem(); 
  private double state = 0;
  private double armMiddlePos;
  private double armBasePos;
  private double SET_ANGLE_Temp;
  private double SET_POWER_Temp;
  private double prevT;
  private double lastT;
  /** Creates a new L2Command. */
  public L2Command(ArmSubsystem m_arm, GroundIntakeSubsystem m_ground) {
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
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == 0.5){
      armMiddlePos = m_arm.getMiddleCANPos();
      if(armMiddlePos < SET_ANGLE_Temp || armBasePos < SET_ANGLE_Temp) {
        if(armMiddlePos < SET_ANGLE_Temp) {
          m_arm.setMiddlePos(SET_ANGLE_Temp);
        }
        if(armBasePos < SET_ANGLE_Temp) {
          m_arm.setRightBasePos(SET_ANGLE_Temp);
        }
        state = 1;
      }
    }
    prevT = Timer.getFPGATimestamp();
    if(armMiddlePos<SET_ANGLE_Temp  && armBasePos<SET_ANGLE_Temp && state == 1) {    
      ArmSubsystem.shootFlag = true;
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
