// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;

public class IntakeShooter extends SubsystemBase {
  /** Creates a new IntakeShooter. */

  private static IntakeShooter mInstance;

  private SparkMax mShooterMotor1;
  private SparkMax mShooterMotor2;

  private TalonFX talonFX;
  private DutyCycleOut mOutput = new DutyCycleOut(0);

  private RelativeEncoder mShooterEncoder;

  private double mSp;
  private PIDController mShooterPID;
  private SimpleMotorFeedforward mFeedForward;
  private boolean isEnable;

  public IntakeShooter() {
    mShooterMotor1 = new SparkMax(FuelConstants.kShooterMotor1, FuelConstants.kType);
    mShooterMotor2 = new SparkMax(FuelConstants.kShooterMotor2, FuelConstants.kType);
    talonFX = new TalonFX(FuelConstants.kDirection);

    TalonFXConfigurator talonFXConfigurator = talonFX.getConfigurator();
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    talonFXConfigs.CurrentLimits.withStatorCurrentLimit(Amps.of(120)).withStatorCurrentLimitEnable(true);

    talonFXConfigurator.apply(talonFXConfigs);

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig reversedConfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(FuelConstants.kCurrentLimit);
    reversedConfig.apply(globalConfig).inverted(true);
    //mShooterMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mShooterMotor1.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mShooterMotor2.configure(reversedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mShooterEncoder = mShooterMotor1.getEncoder();

    mShooterPID = new PIDController(FuelConstants.kP, 0, FuelConstants.KD);
    mFeedForward = new SimpleMotorFeedforward(FuelConstants.kS, FuelConstants.kV);
    mShooterPID.setTolerance(1);
  }

  public void setIntake(double voltage){
    talonFX.setControl(mOutput.withOutput(voltage));
  }

  public void setShooter(double voltage){
    //mShooterMotor.set(voltage);
    mShooterMotor1.set(voltage);
    mShooterMotor2.set(voltage);
  }

  public void stop(){
    mShooterMotor1.set(0);
    mShooterMotor2.set(0);
    talonFX.setControl(mOutput.withOutput(0));
    //mShooterMotor.set(0);
  }

  public double getShooterVelocity(){
    return mShooterEncoder.getVelocity();
  }

  public void enablePID(){
    isEnable = true;
  }

  public void disablePID(){
    isEnable = false;
    mShooterPID.setSetpoint(0);
    mSp = 0;
  }

  public void setSetPoint(double sp){
    mShooterPID.setSetpoint(sp);
    mSp = sp;
  }

  public boolean isSp(){
    return mShooterPID.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double output = 0;
    double PIDShot = 0;
    double ffShot = 0;

    if(isEnable && mSp > 0){
      PIDShot = mShooterPID.calculate(getShooterVelocity(), mSp); 
      ffShot = mFeedForward.calculate(mSp);
      output = PIDShot + ffShot;
    }else{
      output = 0;
      //mShooterPID.reset();
    }

    output = MathUtil.clamp(output, -6, 6);
    //mShooterMotor.setVoltage(output);

    SmartDashboard.putNumber("Shooter velocity", talonFX.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter PID", PIDShot);
    SmartDashboard.putNumber("Shooter FF", ffShot);
    SmartDashboard.putNumber("outout", output);
    SmartDashboard.putNumber("Setpoint", mShooterPID.getSetpoint());
    SmartDashboard.putNumber("Falcon amps", talonFX.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Is setpoint", mShooterPID.atSetpoint());
  }

  public static IntakeShooter getInstance(){
    if(mInstance == null){
      mInstance = new IntakeShooter();
    }
    return mInstance;
  }
}
