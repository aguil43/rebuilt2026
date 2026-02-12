// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpRobot extends Command {
  /** Creates a new UpRobot. */

  private Climber mClimber;
  private Timer mTmr;

  public UpRobot(Climber clim){
    // Use addRequirements() here to declare subsystem dependencies.
    this.mClimber = clim;
    addRequirements(mClimber);
    mTmr = new Timer();
    mTmr.stop();
    mTmr.reset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mClimber.setCoast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Is State", mClimber.getEndPosition() >= -130);
    mClimber.move(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimber.stop();
    mClimber.setBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(mClimber.getEndPosition() <= -130){
      return true;
    }
    return false;
  }
}
