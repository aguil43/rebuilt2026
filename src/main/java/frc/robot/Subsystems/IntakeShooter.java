// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;

public class IntakeShooter extends SubsystemBase {
  /** Creates a new IntakeShooter. */
  private static IntakeShooter mInstance;

  private TalonFX mShooterMotor1;
  private TalonFX mShooterMotor2;

  private TalonFX talonFX;
  private DutyCycleOut mOutput = new DutyCycleOut(0);

  public IntakeShooter() {
    mShooterMotor1 = new TalonFX(FuelConstants.kShooterMotor1);
    mShooterMotor2 = new TalonFX(FuelConstants.kShooterMotor2);
    talonFX = new TalonFX(FuelConstants.kDirection);

    TalonFXConfigurator talonFXConfigurator = talonFX.getConfigurator();
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    talonFXConfigs.CurrentLimits.withStatorCurrentLimit(Amps.of(120)).withStatorCurrentLimitEnable(true);
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXConfiguration talonFXConfigsReverse = new TalonFXConfiguration();
    talonFXConfigsReverse.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    talonFXConfigsReverse.CurrentLimits.withStatorCurrentLimit(Amps.of(120)).withStatorCurrentLimitEnable(true);
    talonFXConfigsReverse.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfigurator.apply(talonFXConfigs);
    mShooterMotor1.getConfigurator().apply(talonFXConfigs);
    mShooterMotor2.getConfigurator().apply(talonFXConfigsReverse);
  }

  public void setIntake(double voltage){
    talonFX.setControl(mOutput.withOutput(voltage));
  }

  public void setShooter(double voltage){
    //mShooterMotor.set(voltage);
    mShooterMotor1.setControl(mOutput.withOutput(voltage));
    mShooterMotor2.setControl(mOutput.withOutput(voltage));
  }

  public void stop(){
    mShooterMotor1.setControl(mOutput.withOutput(0));
    mShooterMotor2.setControl(mOutput.withOutput(0));
    talonFX.setControl(mOutput.withOutput(0));
    //mShooterMotor.set(0);
  }

  public double getShooterVelocity(){
    return mShooterMotor1.getVelocity().getValueAsDouble();
    //return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/Velocity", getShooterVelocity());
    SmartDashboard.putNumber("Indexer/Velocity", talonFX.getVelocity().getValueAsDouble());
  }

  public static IntakeShooter getInstance(){
    if(mInstance == null){
      mInstance = new IntakeShooter();
    }
    return mInstance;
  }
}
