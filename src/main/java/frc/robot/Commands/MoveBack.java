// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveBack extends Command {
  /** Creates a new MoveBack. */

  private Drivetrain mDrive;

  public MoveBack(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mDrive = drive;
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mDrive.getRightAvgPose() > -40){
      mDrive.setSpeeds(-2, -2);
    }else{
      mDrive.setSpeeds(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.setSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(mDrive.getRightAvgPose() <= -24){
      return true;
    }
    return false;
  }
}
