// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DropFuel extends Command {
  /** Creates a new DropFuel. */

  private IntakeShooter mFuel;
  private Timer tmr = new Timer();

  public DropFuel(IntakeShooter fuel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mFuel = fuel;
    addRequirements(mFuel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //mFuel.setSetPoint(1250);
    //mFuel.enablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mFuel.setShooter(0.9);
    if(mFuel.getShooterVelocity() > 2200){
      mFuel.setIntake(0.8);
    }
    //mFuel.setIntake(0.9);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mFuel.stop();
    //mFuel.disablePID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
