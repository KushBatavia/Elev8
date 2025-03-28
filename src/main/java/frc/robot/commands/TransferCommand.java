// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.GroundIntakeSubsystem;
// import frc.robot.subsystems.SparkMaxSubsystem;   

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class TransferCommand extends Command {
//   /** Creates a new TransferCommand. */
//   private ArmSubsystem m_arm = new ArmSubsystem();
//   private GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem(); 
//   private SparkMaxSubsystem m_spark ;
//   private double state = 0;
//   private double armMiddlePos;
//   private double armBasePos;
//   private double SET_ANGLE_Temp;
//   private double SET_POWER_TEMP;
//   private boolean returnFlag;
  
//   public TransferCommand(ArmSubsystem m_arm, GroundIntakeSubsystem m_ground, SparkMaxSubsystem m_spark) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.m_arm = m_arm;
//     this.m_ground = m_ground;
//     this.m_spark = m_spark;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     Constants.killFlag = false;
//     returnFlag = false;
//     ArmSubsystem.algaeFlag = false;
//     if(GroundIntakeSubsystem.intakeState == -1){
//       m_ground.setPos(SET_ANGLE_Temp);
//       state = 1;
//     }else{
//       returnFlag = true;
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(state == 1){
//       m_arm.setRightBasePos(SET_ANGLE_Temp);     
//       m_arm.setMiddlePos(SET_ANGLE_Temp); 
//       m_spark.setArmIntakeMotor(-0.6);
//       state = 2;
//     }
//     else if(state == 2){
//       m_ground.setPos(SET_ANGLE_Temp);
//       state = 3;
//     }

//     else if(state == 3){
//       m_ground.setBottomMotor(SET_ANGLE_Temp);
//     }
    
//     else if(state == 3 && m_arm.getBeamIntake()) {
//       m_arm.setMiddlePos(348);
//       m_arm.setSourcePower(SET_POWER_TEMP);
//       m_ground.setBottomMotor(0);
//       state = 4;
//     }
    
//     else if(state == 4 && m_arm.getBeamIntake() && m_arm.getBeamOuttake()){
//       m_arm.setSourcePower(SET_POWER_TEMP);
//       state = 5;
//     } 
    
//     else if(state == 5 && m_arm.getBeamIntake() && !m_arm.getBeamOuttake()){
//       m_arm.setSourcePower(0);
//       state = 6;
//       returnFlag = true;
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
