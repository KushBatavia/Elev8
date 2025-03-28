// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundCoralCommand extends Command {
  /** Creates a new GroundCoralCommand. */
  private GroundIntakeSubsystem m_ground;
  private ArmSubsystem m_arm;
  private double state = 0;
  private boolean returnFlag;

  private double SET_POWER_TEMP;
  private double SET_ANGLE_Temp;

  public GroundCoralCommand(GroundIntakeSubsystem m_ground, ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ground = m_ground;
    this.m_arm = m_arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.killFlag = false;
    returnFlag = false;
    if(GroundIntakeSubsystem.intakeState == 1){
      if(m_arm.getRightBasePos() > 220) { 
        m_arm.setRightBasePos(218);
      }
        state = 2;
    }else{
      returnFlag = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("chodu sala", state);
    if(state == 2 && m_arm.getRightBasePos()<220){
      m_ground.setPos(97);
      m_ground.setBottomMotor(SET_POWER_TEMP);
      m_ground.setTopMotor(SET_POWER_TEMP);
      state = 3;
    }
    if(state ==3 && m_ground.getTopBeam()){
      m_ground.setBottomMotor(0);
      state = 4;
    }
    if(state ==4){ //VARSHIL SIR WOULD BE PROUD
      state = 5;
    }
    if(state == 5 && m_arm.getBeamIntake()) {
      m_arm.setMiddlePos(348);
      m_arm.setSourcePower(SET_POWER_TEMP);
      state = 6;
    }
    if(state == 6 && m_arm.getBeamIntake() && m_arm.getBeamOuttake()){
      m_ground.setBottomMotor(0);
      m_ground.setTopMotor(0);
      m_arm.setSourcePower(-SET_POWER_TEMP); //reverse direction so it doesn't overshoot
      state = 7;
    } 
    if(state ==7 && m_arm.getBeamIntake() && !m_arm.getBeamOuttake()){
      m_arm.setSourcePower(0);
      state = 8;
      GroundIntakeSubsystem.intakeState = GroundIntakeSubsystem.intakeState*-1;
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
