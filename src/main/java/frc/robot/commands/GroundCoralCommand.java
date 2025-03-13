// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundCoralCommand extends Command {
  /** Creates a new GroundCoralCommand. */
  private GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem();
  private double state = 0;
  private double SET_ANGLE_Temp;
  private double SET_POWER_Temp;
  private boolean returnFlag;
  public GroundCoralCommand(GroundIntakeSubsystem m_ground) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ground = m_ground;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.killFlag = false;
    returnFlag = false;
    // if(GroundIntakeSubsystem.intakeState == 1){
      state = 2;
    // }else{
    //   returnFlag = true;
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("state", state);
    if(state == 2){
      m_ground.setPos(110);
      state = 3;
      GroundIntakeSubsystem.coralState = true;
      SmartDashboard.putNumber("state", state);
    }
    if(m_ground.getHoodPos()<170&&state ==3){
        m_ground.setIntakeMotor(0.3);
        state =4;
      // }else{
      //   m_ground.setIntakeMotor(0);
      }
    if(state ==4 && m_ground.getCurrent()>25){
      m_ground.setIntakeMotor(0);
      m_ground.setPos(160);
      state = 5;
      // GroundIntakeSubsystem.intakeState = GroundIntakeSubsystem.intakeState*-1;
    }
    if(state == 5){
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
