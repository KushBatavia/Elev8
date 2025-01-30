// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmShooterCommand extends Command {
  private ArmSubsystem m_arm = new ArmSubsystem();
  private double state = 0;
  private double prevT;
  private double lastT;
  private double SET_ANGLE_Temp;
  private double SET_POWER_Temp;
  /** Creates a new ArmShooterCommand. */
  public ArmShooterCommand(ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    prevT = Timer.getFPGATimestamp();
    if(m_arm.shootFlag && state == 0){
      m_arm.setArmIntakeMotor(SET_POWER_Temp);
      lastT = Timer.getFPGATimestamp();
      state = 1;
    }
    if(state == 1 && Math.abs(prevT - lastT) > 0.5) {
      m_arm.setArmIntakeMotor(0);
      state = 2; //FIGURE OUT CLOSE CODE FROM HERE IDK HOW THE MECHANISM IS SUPPOSED TO WORK
    }
    if(state == 2){
      m_arm.setMiddlePos(SET_ANGLE_Temp);
      m_arm.setRightBasePos(SET_ANGLE_Temp);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
