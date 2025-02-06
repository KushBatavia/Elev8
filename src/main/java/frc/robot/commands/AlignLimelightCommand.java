package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

public class AlignLimelightCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final double targetX, targetY, targetTheta; 
    private final PIDController xController = new PIDController(0.1, 0, 0);
    private final PIDController yController = new PIDController(0.1, 0, 0);
    private final PIDController thetaController = new PIDController(0.05, 0, 0);

    public AlignLimelightCommand(CommandSwerveDrivetrain drivetrain, double targetX, double targetY, double targetTheta) {
        this.drivetrain = drivetrain;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetTheta = targetTheta;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        
        if (targetPose.length < 6) {
            drivetrain.setControl(new SwerveRequest.RobotCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
            return;
        }

        double currentX = targetPose[0]; // Forward/backward
        double currentY = targetPose[1]; // Left/right
        double currentTheta = targetPose[5]; // Rotation (Yaw)

        double speedX = xController.calculate(currentX, targetX);
        double speedY = yController.calculate(currentY, targetY);
        double speedTheta = thetaController.calculate(currentTheta, targetTheta);

        drivetrain.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(speedX)
            .withVelocityY(speedY)
            .withRotationalRate(speedTheta) // Align rotation
        );
    }

    @Override
    public boolean isFinished() {
        return Math.abs(xController.getPositionError()) < 0.1 &&
               Math.abs(yController.getPositionError()) < 0.1 &&
               Math.abs(thetaController.getPositionError()) < 2.0; // Rotation within 2 degrees
    }
}
