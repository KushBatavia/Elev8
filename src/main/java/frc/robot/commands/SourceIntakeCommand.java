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
public class SourceIntakeCommand extends Command {
  private ArmSubsystem m_arm = new ArmSubsystem();
  private GroundIntakeSubsystem m_groundIntake = new GroundIntakeSubsystem();
  private SparkMaxSubsystem m_spark = new SparkMaxSubsystem();
  private double state = 0;
  private double SET_ANGLE_Temp;
  private double SET_POWER_Temp;
  private double prevT;
  private double lastT;
  private boolean returnFlag;
  
  /** Creates a new SourceIntakeCommand. */
  public SourceIntakeCommand(ArmSubsystem m_arm, GroundIntakeSubsystem m_ground, SparkMaxSubsystem m_spark) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_groundIntake = m_ground;
    this.m_spark = m_spark;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    returnFlag = false;
    ArmSubsystem.algaeFlag = false;
    if(m_groundIntake.getPos()>255 || m_groundIntake.getPos()<200) {
      state = 4;
    }else{
      state = 1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == 1){
      m_arm.setRightBasePos(SET_ANGLE_Temp);
      state =2;
    }
    if(state ==2 && m_arm.getRightBasePos()<SET_ANGLE_Temp){
      m_groundIntake.setPos(SET_ANGLE_Temp);
      state = 3;
    }

    if(state == 3 && m_groundIntake.getPos()<SET_ANGLE_Temp){
      m_arm.setMiddlePos(SET_ANGLE_Temp);
      m_arm.setRightBasePos(SET_ANGLE_Temp);
      ArmSubsystem.armState = 5;
      state = 5;
    }
    if(m_arm.getMiddlePos()<SET_ANGLE_Temp  &&  m_arm.getRightBasePos()<SET_ANGLE_Temp && state == 5) {
      m_spark.setArmIntakeMotor(SET_POWER_Temp);
      state = 6;
    }
    if(state == 6 && m_arm.getBeamIntake()&&m_arm.getBeamOuttake()) {
      m_spark.setArmIntakeMotor(0);
      state = 7;
    if(state == 7){
      m_spark.setArmIntakeMotor(SET_POWER_Temp);
      state = 8;
    } 
    if(state ==8 && !m_arm.getBeamIntake() && m_arm.getBeamOuttake()){
      m_spark.setArmIntakeMotor(0);
      state = 9;
      returnFlag = true;
    }
      
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
