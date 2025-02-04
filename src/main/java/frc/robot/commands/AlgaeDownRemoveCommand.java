// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.SparkMaxSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeDownRemoveCommand extends Command {
  private ArmSubsystem m_arm = new ArmSubsystem();
  private GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem();
  private SparkMaxSubsystem m_spark = new SparkMaxSubsystem();
  private double state = 0;
  private double armMiddlePos;
  private double armBasePos;
  private double SET_ANGLE_Temp; //I dont have values, so these are just temporary variable created that im using everywhere.
  private double SET_POWER_Temp;//Not the same everywhere, just follow the logic dont look at the variable being used
  private double prevT;
  private double lastT;
  private boolean returnFlag;

  /** Creates a new AlgaeDownRemoveCommand. */
  public AlgaeDownRemoveCommand(ArmSubsystem m_arm, GroundIntakeSubsystem m_ground, SparkMaxSubsystem m_spark) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_ground = m_ground;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    returnFlag = false;
    if(m_ground.getPos()>SET_ANGLE_Temp && m_ground.getPos()<SET_ANGLE_Temp) {
      state = 0.5;
    }else{
      returnFlag = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == 0.5){
      //SET THE MOTORS TO THE STARTING VALUES
      if(m_arm.getMiddleCANPos() < SET_ANGLE_Temp || m_arm.getRightBaseCANPos() < SET_ANGLE_Temp) {
        if(m_arm.getMiddleCANPos() < SET_ANGLE_Temp) {
          m_arm.setMiddlePos(SET_ANGLE_Temp);
        }
        if(m_arm.getRightBaseCANPos() < SET_ANGLE_Temp) {
          m_arm.setRightBasePos(SET_ANGLE_Temp);
        }
        state = 1; 
        ArmSubsystem.algaeFlag = true;
      }    
      returnFlag = true;
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return returnFlag;
  }
}
