// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CompoundLeave extends Command {
  private static double waitTime = 1.5;
  private static double initTime;
  private static double currentTime;
  private static int state = 0;
  private boolean returnFlag = false;
  private int L = 1;
  
  /** Creates a new Grab. */
  public CompoundLeave(int L) {
    this.L = L;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    returnFlag = false;
    state=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();
    if (state == 0)
    {
      initTime = Timer.getFPGATimestamp();
      if (L==1) { ElevatorSubsystem.L1(); }
      else if (L==2) { ElevatorSubsystem.L2(); }
      else if (L==3) { ElevatorSubsystem.L3(); }
      //else if (L==4) { ElevatorSubsystem.L4(); }
      state = 1;
    }
    if (state==1 && Math.abs(currentTime - initTime)>=1)
    {
      initTime = Timer.getFPGATimestamp();
      ElbowSubsystem.SetDrop();
      state=2;
    }
    if (state == 2 && Math.abs(currentTime - initTime)>=(0.5)*L)
    {
      GrabberSubsystem.RunGrabber(-0.3);
      initTime = Timer.getFPGATimestamp();
      state=3;
    }
    if (state==3)
    {
      if (Math.abs(currentTime - initTime)>=waitTime /*| !GrabberSubsystem.GetBeam()*/)
      {
        state = 4;
      }
    }    
    if (state==4)
    {
      GrabberSubsystem.RunGrabber(0);
      returnFlag = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return returnFlag;
  }
}
