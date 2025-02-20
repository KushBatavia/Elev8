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
    Constants.killFlag = false;
    // state = 0;
    returnFlag = false;
    ArmSubsystem.algaeFlag = false;
    if(m_groundIntake.getPos()>287 /*change later*/) {
      m_groundIntake.setPos(262);
      state = 4;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_groundIntake.getPos() > 257 && state==4){
      m_arm.setMiddlePos(385);
      m_arm.setRightBasePos(253);
      m_spark.setArmIntakeMotor(-0.6);
      ArmSubsystem.armState = 5;
      state = 5;
    }
    if(state == 5 && m_arm.getBeamIntake()) {
      m_arm.setMiddlePos(348);
      m_spark.setArmIntakeMotor(-0.35);
      state = 6;
    }
    if(state == 6 && m_arm.getBeamIntake() && m_arm.getBeamOuttake()){
      m_spark.setArmIntakeMotor(0.2);
      state = 7;
    } 
    if(state ==7 && m_arm.getBeamIntake() && !m_arm.getBeamOuttake()){
      m_spark.setArmIntakeMotor(0);
      state = 8;
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
