// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlgaeDownRemoveCommand;
import frc.robot.commands.AlgaeUpRemovalCommand;
import frc.robot.commands.ArmShooterCommand;
import frc.robot.commands.GroundAlgaeCommand;
import frc.robot.commands.GroundCoralCommand;
import frc.robot.commands.GroundOuttakeCommand;
import frc.robot.commands.HangClimbCommand;
import frc.robot.commands.HangPositionCommand;
import frc.robot.commands.L2Command;
import frc.robot.commands.L3Command;
import frc.robot.commands.SourceIntakeCommand;
import frc.robot.commands.StrafeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.SparkMaxSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    

    public ArmSubsystem armSubsystem = new ArmSubsystem();
    public GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();
    public SparkMaxSubsystem sparkMax = new SparkMaxSubsystem();
    // public ArmIntakeCommand armCommand = new ArmIntakeCommand(armSubsystem);
    // public GroundCoralCommand groundCoralCommand = new GroundCoralCommand(groundIntake);
    // public GroundAlgaeCommand groundAlgaeCommand = new GroundAlgaeCommand(groundIntake);
    // public GroundOuttakeCommand groundOuttakeCommand = new GroundOuttakeCommand(groundIntake);
    // public L2Command l2Command = new L2Command(armSubsystem);
    // public ArmIntakeCommand armIntakeCommand = new ArmIntakeCommand(armSubsystem);
    // public ArmOuttakeCommand armOuttakeCommand = new ArmOuttakeCommand(armSubsystem);
    // public AlgaeRemovalUpperCommand algaeRemovalUpperCommand = new AlgaeRemovalUpperCommand(armSubsystem);
    // public AlgaeRemovalDownerCommand algaeRemovalDownerCommand = new AlgaeRemovalDownerCommand(armSubsystem);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // private SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
    //         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate*0.1)
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RobotContainer() {
        configureBindings();
    }
    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        if ((Math.abs(joystick.getLeftX()) > 0.01) && (Math.abs(joystick.getLeftX()) > 0.01)) { 
            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );
        }

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Testing
        joystick.a().onTrue(new InstantCommand(() -> {
            // armSubsystem.setMiddlePos(95);
            // armSubsystem.setRightBasePos(230);
        }));
        
        joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        joystick.a().onTrue(new GroundCoralCommand(groundIntake, armSubsystem));
        joystick.x().onTrue(new GroundOuttakeCommand(groundIntake, armSubsystem));
        joystick.b().onTrue(new L2Command(armSubsystem, groundIntake, sparkMax));
        joystick.y().onTrue(new L3Command(armSubsystem, groundIntake, sparkMax));
        //joystick.rightBumper().onTrue(new SourceIntakeCommand(armSubsystem, groundIntake, sparkMax));
        //joystick.leftBumper().onTrue(new ArmShooterCommand(armSubsystem, sparkMax));
        joystick.pov(90).whileTrue(new StrafeCommand(drivetrain, joystick));
        joystick.pov(270).whileTrue(new StrafeCommand(drivetrain, joystick));
        joystick.pov(180).onTrue(new HangClimbCommand(armSubsystem, groundIntake));
        //check for algae removal upper and downer

        joystick2.a().onTrue(new GroundCoralCommand(groundIntake, armSubsystem));
        joystick2.leftBumper().onTrue(new GroundAlgaeCommand(groundIntake));
        joystick2.rightBumper().onTrue(new SourceIntakeCommand(armSubsystem, groundIntake, sparkMax));
        joystick2.y().onTrue(new AlgaeUpRemovalCommand(armSubsystem));
        joystick2.x().onTrue(new AlgaeDownRemoveCommand(armSubsystem));
        joystick2.pov(180).onTrue(new HangClimbCommand(armSubsystem, groundIntake));
        joystick2.pov(0).onTrue(new HangPositionCommand(armSubsystem, groundIntake));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
