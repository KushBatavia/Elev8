// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StrafeCommand extends Command {
  /** Creates a new StrafeCommand. */
  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController joystick;
  private boolean leftDPad = false;
  private boolean rightDPad = false;
  private boolean returnFlag;

  public StrafeCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.joystick = joystick;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    returnFlag = false;
    Constants.killFlag = false;
  }

  public void execute() {
    joystick.pov(90).whileTrue(new InstantCommand(() -> {
        leftDPad = true;
    }));
    joystick.pov(270).whileTrue(new InstantCommand(() -> {
        rightDPad = true;
    }));
    
    if (leftDPad) {
        drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0.1));
    } else if (rightDPad) {
        drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0.1));
    } else if(!leftDPad && !rightDPad) {
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
