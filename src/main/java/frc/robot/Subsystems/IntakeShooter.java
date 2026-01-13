// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;

public class IntakeShooter extends SubsystemBase {
  /** Creates a new IntakeShooter. */

  private static IntakeShooter mInstance;

  private SparkMax mShooterMotor;
  private SparkMax mIntakeMotor;

  private RelativeEncoder mShooterEncoder;
  private RelativeEncoder mIntakeEncoder;

  public IntakeShooter() {
    mShooterMotor = new SparkMax(FuelConstants.kShooter, FuelConstants.kType);
    mIntakeMotor = new SparkMax(FuelConstants.kDirection, FuelConstants.kType);

    SparkMaxConfig globalConfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(FuelConstants.kCurrentLimit);

    mShooterMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mIntakeMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mShooterEncoder = mShooterMotor.getEncoder();
    mIntakeEncoder = mIntakeMotor.getEncoder();
  }

  public void setIntake(double voltage){
    mIntakeMotor.set(voltage);
  }

  public void setShooter(double voltage){
    mShooterMotor.set(voltage);
  }

  public void stop(){
    mIntakeMotor.set(0);
    mShooterMotor.set(0);
  }

  public double getShooterVelocity(){
    return mShooterEncoder.getVelocity();
  }

  public double getIntakeVelocity(){
    return mIntakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter velocity", getShooterVelocity());
    SmartDashboard.putNumber("Intake velocity", getIntakeVelocity());
  }

  public static IntakeShooter getInstance(){
    if(mInstance == null){
      mInstance = new IntakeShooter();
    }
    return mInstance;
  }
}
