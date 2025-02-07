package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

public class AlignLimelightCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private boolean alignX, alignY, alignTheta;
    private double tx, ty, targetYaw, robotYaw;
    private boolean returnFlag = false;
    private final PIDController xController = new PIDController(0.1, 0, 0);
    private final PIDController yController = new PIDController(0.1, 0, 0);
    private final PIDController thetaController = new PIDController(0.05, 0, 0); //Tune all of these later
    private final Pigeon2 pigeon = new Pigeon2(0); //Double check CAN Ids later
    private double offsetX;


    @Override
    public void initialize() {
        returnFlag = false;
    }

    public AlignLimelightCommand(CommandSwerveDrivetrain drivetrain, boolean alignX, boolean alignY, boolean alignTheta, double offsetX) {
        this.drivetrain = drivetrain;
        this.alignX = alignX;
        this.alignY = alignY;
        this.alignTheta = alignTheta;
        this.offsetX = offsetX;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!alignTheta) { robotYaw = 0; targetYaw = 0;}
        else { targetYaw = LimelightHelpers.getBotPose2d_wpiBlue("limelight-new").getRotation().getDegrees();
        robotYaw = pigeon.getYaw().getValueAsDouble(); /*not sure about this one */}
        if (!alignX) { tx = 0;} 
        else { tx = LimelightHelpers.getTX("limelight-new");}
        if (!alignY) { ty = 0;} 
        else {ty = LimelightHelpers.getTY("limelight-new");}

        double thetaError = targetYaw - robotYaw;
        double speedX = xController.calculate(tx, offsetX);
        double speedY = yController.calculate(ty, 0);
        double speedTheta = thetaController.calculate(thetaError, 0);

        drivetrain.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(speedX) // Forward/Backward
            .withVelocityY(-speedY) // Left/Right
            .withRotationalRate(speedTheta) // Align rotation
        );

        if (Math.abs(xController.getPositionError()) < 0.1 && Math.abs(yController.getPositionError()) < 0.1 &&  Math.abs(thetaController.getPositionError()) < 2.0)
        {
            returnFlag = true;
        }
    }

    @Override
    public boolean isFinished() {
        return returnFlag;
    }
}
