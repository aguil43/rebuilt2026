// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private static Drivetrain mDrivetrain;

  private SparkMax mLeftBack;
  private SparkMax mLeftFront;
  private SparkMax mRightBack;
  private SparkMax mRightFront;

  private RelativeEncoder mLeftBackMotorEncoder;
  private RelativeEncoder mLeftFrontMotorEncoder;
  private RelativeEncoder mRightBackMotorEncoder;
  private RelativeEncoder mRightFrontMotorEncoder;

  private DifferentialDrive mDrive;

  public Drivetrain() {
    mLeftBack = new SparkMax(DriveConstants.kLeftBack, DriveConstants.kType);
    mLeftFront = new SparkMax(DriveConstants.kLeftFront, DriveConstants.kType);
    mRightBack = new SparkMax(DriveConstants.kRightBack, DriveConstants.kType);
    mRightFront = new SparkMax(DriveConstants.kRightFront, DriveConstants.kType);

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leftMasterConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightMasterConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(DriveConstants.kCurrentLimit).idleMode(IdleMode.kCoast);
    leftMasterConfig.apply(globalConfig).inverted(true);
    rightMasterConfig.apply(globalConfig);
    leftFollowerConfig.apply(globalConfig).follow(mLeftBack);
    rightFollowerConfig.apply(globalConfig).follow(mRightBack);

    mLeftBack.configure(leftMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mLeftFront.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mRightBack.configure(rightMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mRightFront.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mDrive = new DifferentialDrive(mLeftBack, mRightBack);
    mDrive.setSafetyEnabled(false);

    mLeftBackMotorEncoder = mLeftBack.getEncoder();
    mLeftFrontMotorEncoder = mLeftFront.getEncoder();
    mRightBackMotorEncoder = mRightBack.getEncoder();
    mRightFrontMotorEncoder = mRightFront.getEncoder();
  }

  public void teleopDrive(double xVel, double zVel){
    mDrive.arcadeDrive(xVel * DriveConstants.kVelLimit, zVel * DriveConstants.kRotLimit, true);
  }

  public void autoDrive(double leftVel, double rightVel){
    mDrive.tankDrive(leftVel, rightVel, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Output left", mLeftBack.getAppliedOutput());
    SmartDashboard.putNumber("Output right", mRightBack.getAppliedOutput());

    SmartDashboard.putNumber("Left encoder back", mLeftBackMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Left encoder front", mLeftFrontMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Right encoder back", mRightBackMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Right encoder front", mRightFrontMotorEncoder.getVelocity());
  }

  public static Drivetrain getInstance(){
    if(mDrivetrain == null){
      mDrivetrain = new Drivetrain();
    }
    return mDrivetrain;
  }
}
