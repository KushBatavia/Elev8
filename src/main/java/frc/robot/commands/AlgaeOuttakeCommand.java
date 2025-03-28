// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeOuttakeCommand extends Command {
  private ArmSubsystem m_arm;
  private double state;
  private double prevT;
  private double lastT;
  private double SET_ANGLE_Temp; //I dont have values, so these are just temporary variable created that im using everywhere.
  private double SET_POWER_Temp;//Not the same everywhere, just follow the logic dont look at the variable being used
  private boolean returnFlag;


  /** Creates a new AlgaeRemoveRunCommand. */
  public AlgaeOuttakeCommand(ArmSubsystem m_arm){
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.killFlag = false;
    state = 0;
    returnFlag = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == 0) {
      m_arm.setSourcePower(SET_ANGLE_Temp);
      state = 1;
      prevT = Timer.getFPGATimestamp();
    }
    lastT = Timer.getFPGATimestamp();
    if(state == 1 && Math.abs(prevT-lastT)>0.5) {
        m_arm.setSourcePower(0);
        state = 2;
      }
      returnFlag = true;
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
