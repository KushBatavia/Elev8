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
    public boolean alignY;
    private double tx;
    private double ty;
    private boolean returnFlag = false;
    private final PIDController xController = new PIDController(0.1, 0, 0);
    private final PIDController yController = new PIDController(0.1, 0, 0);


    @Override
    public void initialize() {
        alignX = true;
        alignY = true;
        returnFlag = false;
    }

    public AlignLimelightCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        if (!alignX && !alignY) { tx = 0; ty = 0; } 
        else { tx = LimelightHelpers.getTX("limelight-new"); ty = LimelightHelpers.getTY("limelight-new");}

        double speedX = xController.calculate(tx, -10);
        double speedY = yController.calculate(ty, 0);

        drivetrain.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(speedX).withVelocityY(speedY) // Forward/Backward
        );

        if (Math.abs(xController.getPositionError()) < 2) /*degrees */ {
            alignX = false;
            alignY = false; 
            returnFlag = true;
        }
    }

    @Override
    public boolean isFinished() {
        return returnFlag;
    }
}
