package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

public class AlignLimelightCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private boolean alignX;
    private double tx;
    private boolean returnFlag = false;
    private final PIDController xController = new PIDController(0.1, 0, 0);


    @Override
    public void initialize() {
        alignX = true;
        returnFlag = false;
    }

    public AlignLimelightCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        if (!alignX) { tx = 0;} 
        else { tx = LimelightHelpers.getTX("limelight-new");}

        double speedX = xController.calculate(tx, 0);

        drivetrain.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(speedX) // Forward/Backward
        );

        if (Math.abs(xController.getPositionError()) < 2 /*degrees */)
        {
            alignX = false;
            returnFlag = true;
        }
    }

    @Override
    public boolean isFinished() {
        return returnFlag;
    }
}
