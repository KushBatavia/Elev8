// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundCoralCommand extends Command {
  /** Creates a new GroundCoralCommand. */
  private GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem();
  private ArmSubsystem m_arm = new ArmSubsystem();
  private double state = 0;
  private double SET_ANGLE_Temp;
  private double SET_POWER_Temp;
  private boolean returnFlag;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(GroundIntakeSubsystem.intakeState == 1){
      if(m_arm.getRightBaseCANPos() > 230) {
        m_arm.setMiddlePos(345);
        m_arm.setRightBasePos(225);
      }
      state = 2;  
    }else{
      returnFlag = true;
    }
    if(state == 2){
      m_ground.setPos(95);
      m_ground.setIntakeMotor(0.3);
      state = 3;
      GroundIntakeSubsystem.coralState = true;
    }
    if(state ==3 && m_ground.getCurrent()>20){
      m_ground.setIntakeMotor(0);
      m_ground.setPos(203);
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
