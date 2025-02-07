// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.SparkMaxSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L3Command extends Command {
  private ArmSubsystem m_arm = new ArmSubsystem();
  private GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem(); 
  private SparkMaxSubsystem m_spark = new SparkMaxSubsystem();
  private double state = 0;
  private double armMiddlePos;
  private double armBasePos;
  private double SET_ANGLE_Temp;
  private boolean returnFlag;
  /** Creates a new L3Command. */
  public L3Command(ArmSubsystem m_arm, GroundIntakeSubsystem m_ground, SparkMaxSubsystem m_spark) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_ground = m_ground;
    this.m_spark = m_spark;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.killFlag = false;
    ArmSubsystem.algaeFlag = false;
    m_spark.setArmIntakeMotor(0);
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
        m_arm.setRightBasePos(SET_ANGLE_Temp);//HES SET THIS TO THE SAME ANGLE AS L2. WHAT DO I DO NOW
        m_arm.setMiddlePos(SET_ANGLE_Temp);
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
      m_arm.setRightBasePos(190);
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
