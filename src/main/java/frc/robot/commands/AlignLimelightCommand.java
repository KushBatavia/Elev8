package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

public class AlignLimelightCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private boolean alignX;
    private boolean alignY;
    private double tx;
    private double ty;
    private boolean returnFlag = false;
    private final PIDController xController = new PIDController(0.1, 0, 0);
    private final PIDController yController = new PIDController(0.1, 0, 0);
    private final double limelightOffsetX = -10;


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
        if (!alignX) { tx = 0;} 
        else { tx = LimelightHelpers.getTX("limelight-a");}
        double speedX = xController.calculate(tx, limelightOffsetX);
        if (!alignY) { tx = 0;} 
        else { ty = LimelightHelpers.getTY("limelight-a");}
        double speedY = yController.calculate(ty, 0);

        drivetrain.setControl(new SwerveRequest.RobotCentric()
            .withVelocityY(speedX * 0.1) // Left/right
            .withVelocityX(speedY * 0.1) //Front/Back
        );

        if (Math.abs(xController.getPositionError()) < 5 && Math.abs(yController.getPositionError()) < 5)
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
