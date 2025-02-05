// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundOuttakeCommand extends Command {
  /** Creates a new GroundOuttakeCommand. */
  private GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem();
  private ArmSubsystem m_arm = new ArmSubsystem();
  private double state = 0;
  private double SET_ANGLE_Temp;
  private double SET_POWER_Temp;
  private double prevT;
  private double lastT;
  private double basePos;
  private double currentThreshold;
  private boolean returnFlag;
  public GroundOuttakeCommand(GroundIntakeSubsystem m_ground, ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_ground = m_ground;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    returnFlag = false;
    if(m_ground.intakeState == -1){
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
        m_arm.setMiddlePos(SET_ANGLE_Temp);  
        m_arm.setRightBasePos(SET_ANGLE_Temp);
      }
        state = 0.5;
    }
    if(state == 0.5){
      m_ground.setPos(SET_ANGLE_Temp);
      state = 1;
    }
    prevT = Timer.getFPGATimestamp();
    if(state ==1 && m_ground.getPos() > SET_ANGLE_Temp) {
      if (m_ground.getBeamBreaker1()) { 
        //Coral
        m_ground.setIntakeMotor(SET_POWER_Temp);
        m_ground.setPos(SET_ANGLE_Temp);
      } else if (!m_ground.getBeamBreaker1()) { 
        //Algae
        m_ground.setIntakeMotor(SET_POWER_Temp);
        m_ground.setPos(SET_ANGLE_Temp);

      }
      state = 2;
      lastT = Timer.getFPGATimestamp();
    }
    if(state ==2 && Math.abs(lastT - prevT) > 0.2){
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
