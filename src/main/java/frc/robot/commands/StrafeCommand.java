// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StrafeCommand extends Command {
  /** Creates a new StrafeCommand. */
  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController joystick;
  public StrafeCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.joystick = joystick;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftTrigger = joystick.getLeftTriggerAxis();
    double rightTrigger = joystick.getRightTriggerAxis();
    
        if (leftTrigger > 0.1) {
            drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-leftTrigger * 0.5));
        } else if (rightTrigger > 0.1) {
            drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(rightTrigger*0.5));
        } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
