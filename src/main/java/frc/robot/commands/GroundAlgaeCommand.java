// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundAlgaeCommand extends Command {
  private GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem();
  private ArmSubsystem m_arm = new ArmSubsystem();
  private double state = 0;
  private double SET_ANGLE_Temp;
  private double SET_POWER_Temp;
  private double prevT;
  private double lastT;
  private double currentThreshold;
  private boolean returnFlag;
  public GroundAlgaeCommand(GroundIntakeSubsystem m_ground, ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ground = m_ground;
    this.m_arm = m_arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    returnFlag = false;
    if(m_ground.intakeState == 1){
      state = 0.1;
    }else{
      returnFlag = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == 0.1){
      if(m_arm.getMiddleCANPos() < SET_ANGLE_Temp || m_arm.getRightBaseCANPos() < SET_ANGLE_Temp) { 
        //FIGURE OUT THE SIGN FOR THE IF STATEMENT I FORGOT
        m_arm.setMiddlePos(SET_ANGLE_Temp);  
        m_arm.setRightBasePos(SET_ANGLE_Temp);
        state = 0.5;      
      }
      
    }
    if(state == 0.5){
      m_ground.setPos(SET_ANGLE_Temp);
      state = 1;
    }
    prevT = Timer.getFPGATimestamp();
    if(state ==1 && m_ground.getPos() > SET_ANGLE_Temp) {
      m_ground.setIntakeMotor(SET_POWER_Temp);
      lastT = Timer.getFPGATimestamp();
      state = 2;
    }
    if(state == 2 && m_ground.getCurrent() > currentThreshold && Math.abs(prevT - lastT) > 0.5) {
      m_ground.setIntakeMotor(0);
      m_ground.intakeState = m_ground.intakeState*-1;
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
