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
public class GroundOuttakeCommand extends Command {
  /** Creates a new GroundOuttakeCommand. */
  private GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem();
  private ArmSubsystem m_arm = new ArmSubsystem();
  private double state = 0;
  private double SET_ANGLE_Temp;
  private double SET_POWER_Temp;
  private double prevT;
  private double lastT;
  private boolean returnFlag;
  public GroundOuttakeCommand(GroundIntakeSubsystem m_ground, ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_ground = m_ground;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.killFlag = false;
    if(m_ground.getHoodPos()<220 && GroundIntakeSubsystem.intakeState == -1) {
       state = 1; 
       prevT = 0; 
       lastT = 0;
       returnFlag = false;
       Constants.killFlag = false; 

    }else{
      Constants.killFlag = true; 
      returnFlag = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    prevT = Timer.getFPGATimestamp();
    if(state == 1) {
      if (GroundIntakeSubsystem.coralState) { 
        //Coral
        m_ground.setIntakeMotor(-0.19);
        m_ground.setPos(263);
      } else if (!GroundIntakeSubsystem.coralState) { 
        //Algae
        m_ground.setIntakeMotor(-0.8);
      }
      state = 2;
      lastT = Timer.getFPGATimestamp();
    }
    if(state == 2 && Math.abs(lastT - prevT) > 0.5){
      m_ground.setIntakeMotor(0);
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
