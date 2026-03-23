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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */

  private static Hopper mHopper;

  private SparkMax mMotor;
  private RelativeEncoder mEncoder;

  public Hopper() {
    mMotor = new SparkMax(HopperConstants.kMotorHopper, HopperConstants.kMotorType);

    SparkMaxConfig configs = new SparkMaxConfig();

    configs.smartCurrentLimit(HopperConstants.kAmpsLim).idleMode(IdleMode.kBrake);

    mMotor.configure(configs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mEncoder = mMotor.getEncoder();
  }

  public void activate(double voltage){
    mMotor.set(voltage);
  }

  public void stop(){
    mMotor.set(0);
  }

  public double getPosition(){
    return mEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hopper/Position", mEncoder.getPosition());
  }

  public static Hopper getInstance(){
    if(mHopper == null){
      mHopper = new Hopper();
    }
    return mHopper;
  }
}
