// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Commands.DropFuel;
import frc.robot.Commands.FollowAprilTagAuto;
import frc.robot.Commands.GetFuel;
import frc.robot.Constants.RobotConstants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.IntakeShooter;

public class RobotContainer {

  private Drivetrain mDrive = Drivetrain.getInstance();
  private CommandXboxController mController = new CommandXboxController(RobotConstants.kDriveControllerPort);
  private IntakeShooter mFuel = IntakeShooter.getInstance();

  public RobotContainer() {
    configureBindings();
    mDrive.setDefaultCommand(new DefaultDrive(mDrive, mController));
  }

  private void configureBindings() {
    mController.a().whileTrue(new DropFuel(mFuel));
    mController.b().whileTrue(new GetFuel(mFuel));
    //mController.x().whileTrue(new setIntake(-1, mFuel));
    //mController.y().whileTrue(new setIntake(1, mFuel));
  }

  public Command getAutonomousCommand() {
    //return Commands.print("No autonomous command configured");
    return Commands.sequence(
      new FollowAprilTagAuto(),
      new DropFuel(mFuel).withTimeout(10)
    );
  }
}
