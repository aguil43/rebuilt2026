// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowAprilTagAuto extends Command {
  /** Creates a new FollowAprilTag. */

  private Drivetrain mDrive;
  private double tx;
  //private double ty;
  private double ta;
  boolean firstStage;
  boolean secondStage;
  boolean isDetected;
  private int safeArea = 10;

  public FollowAprilTagAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrive = Drivetrain.getInstance();
    addRequirements(mDrive);
    firstStage = false;
    secondStage = false;
    isDetected = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = LimelightHelpers.getTX(VisionConstants.kLimelightName);
    //ty = LimelightHelpers.getTY(VisionConstants.kLimelightName);
    ta = LimelightHelpers.getTA(VisionConstants.kLimelightName);

    double angularIzq, linealIzq, left;
    double angularDer, linealDer, right;

    if(ta < 5 && ta != 0){
      linealIzq = -0.5;
      linealDer = -0.5;
      firstStage = false;
      isDetected = true;
    }else if(ta > 10){
      linealIzq = 0.5;
      linealDer = 0.5;
      firstStage = false;
      isDetected = true;
    }else{
      linealDer = 0;
      linealIzq = 0;
      firstStage = true;
    }

    if(tx > safeArea){
      angularIzq = -0.5;
      angularDer = 0.5;
      secondStage = false;
      isDetected = true;
    }else if(tx < -safeArea){
      angularIzq = 0.5;
      angularDer = -0.5;
      secondStage = false;
      isDetected = true;
    }else{
      angularIzq = 0;
      angularDer = 0;
      secondStage = true;
    }
    
    left = (angularIzq / 2) + linealIzq;
    right = (angularDer / 2) + linealDer;

    mDrive.autoDrive(-left*.75, -right*.75);

    SmartDashboard.putNumber("TA", LimelightHelpers.getTA(VisionConstants.kLimelightName));
    SmartDashboard.putNumber("TX", LimelightHelpers.getTX(VisionConstants.kLimelightName));

    SmartDashboard.putNumber("Left", left);
    SmartDashboard.putNumber("Right", right);

    SmartDashboard.putBoolean("First stage", firstStage);
    SmartDashboard.putBoolean("Second stage", secondStage);
    SmartDashboard.putBoolean("isDetected", isDetected);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(firstStage && secondStage && isDetected){
      return true;
    }
    return false;
  }
}
