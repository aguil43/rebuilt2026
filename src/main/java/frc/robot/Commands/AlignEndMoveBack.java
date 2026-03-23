// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignEndMoveBack extends Command {
  /** Creates a new AlignEndMoveBack. */

  private Drivetrain mDrive;
  private boolean isAligned = false;

  public AlignEndMoveBack(Drivetrain drive) {
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
    if(LimelightHelpers.getTX(VisionConstants.kLimelightName) >= -2){
      mDrive.setSpeeds(-0.5, 0.5);
    }else if(LimelightHelpers.getTX(VisionConstants.kLimelightName) <= -5){
      mDrive.setSpeeds(0.5, -0.5);
    }else{
      mDrive.setSpeeds(0, 0);
      isAligned = true;
    }

    if(isAligned && mDrive.getRightAvgPose() > -36){
      mDrive.setSpeeds(-2, -2);
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
    if(isAligned && mDrive.getRightAvgPose() < -36){
      return true;
    }
    return false;
  }
}
