// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GetFuel extends Command {
  /** Creates a new GetFuel. */

  private IntakeShooter mFuel;

  public GetFuel(IntakeShooter fuel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mFuel = fuel;
    addRequirements(mFuel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //mFuel.setSetPoint(10);
    //mFuel.enablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mFuel.setIntake(0.4);
    mFuel.setShooter(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //mFuel.disablePID();
    mFuel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
