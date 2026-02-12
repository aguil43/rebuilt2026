// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private TalonFX talonFx;
  private DutyCycleOut mOutput;

  private static Climber mClimber;

  public Climber() {
    talonFx = new TalonFX(ClimberConstants.kMotor);

    TalonFXConfigurator config = talonFx.getConfigurator();
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    configs.CurrentLimits.withStatorCurrentLimit(Amps.of(120)).withStatorCurrentLimitEnable(true);

    config.apply(configs);

    mOutput = new DutyCycleOut(0);
  }

  public void move(double voltage){
    talonFx.setControl(mOutput.withOutput(voltage));
  }

  public void stop(){
    talonFx.setControl(mOutput.withOutput(0));
  }

  public void setCoast(){
    talonFx.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrake(){
    talonFx.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getEndPosition(){
    double convert = talonFx.getPosition().getValueAsDouble();
    convert = convert * (1/840); 
    return convert;
  }

  public double getMotorPosition(){
    return talonFx.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber distance", talonFx.getPosition().getValueAsDouble());
  }

  public static Climber getInstance(){
    if(mClimber == null){
      mClimber = new Climber();
    }
    return mClimber;
  }
}
