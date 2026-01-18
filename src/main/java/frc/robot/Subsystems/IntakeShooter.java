// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
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

  private PIDController mShooterPID;
  private boolean isEnable;

  public IntakeShooter() {
    mShooterMotor = new SparkMax(FuelConstants.kShooter, FuelConstants.kType);
    mIntakeMotor = new SparkMax(FuelConstants.kDirection, FuelConstants.kTypeIntake);

    SparkMaxConfig globalConfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(FuelConstants.kCurrentLimit);

    mShooterMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mIntakeMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mShooterEncoder = mShooterMotor.getEncoder();
    mIntakeEncoder = mIntakeMotor.getEncoder();

    mShooterPID = new PIDController(FuelConstants.kP, FuelConstants.kI, FuelConstants.KD);
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

  public void enablePID(){
    isEnable = true;
  }

  public void disablePID(){
    isEnable = false;
  }

  public void setSetPoint(double sp){
    mShooterPID.setSetpoint(sp);
  }

  public boolean isSp(){
    return mShooterPID.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double output = 0;

    if(isEnable){
      output = mShooterPID.calculate(getShooterVelocity());
    }else{
      output = 0;
    }

    mShooterMotor.set(output);

    SmartDashboard.putNumber("Shooter velocity", getShooterVelocity());
    SmartDashboard.putNumber("Intake velocity", getIntakeVelocity());
    SmartDashboard.putNumber("Shooter PID", output);
    SmartDashboard.putNumber("Setpoint", mShooterPID.getSetpoint());
  }

  public static IntakeShooter getInstance(){
    if(mInstance == null){
      mInstance = new IntakeShooter();
    }
    return mInstance;
  }
}
