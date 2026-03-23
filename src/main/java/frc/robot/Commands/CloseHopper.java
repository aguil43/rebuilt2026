// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Hopper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CloseHopper extends Command {
  /** Creates a new CloseHopper. */

  private Hopper mHopper;

  public CloseHopper(Hopper hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mHopper = hopper;
    addRequirements(mHopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mHopper.activate(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mHopper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(mHopper.getPosition() <= -6.5){
      return true;
    }
    return false;
  }
}
