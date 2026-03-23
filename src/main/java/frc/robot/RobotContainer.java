// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AlignEndMoveBack;
import frc.robot.Commands.CloseHopper;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Commands.DownRobot;
import frc.robot.Commands.DropFuel;
import frc.robot.Commands.DropFuelDown;
import frc.robot.Commands.FollowAprilTagAuto;
import frc.robot.Commands.GetFuel;
import frc.robot.Commands.MoveBack;
import frc.robot.Commands.OpenHopper;
import frc.robot.Commands.UpRobot;
import frc.robot.Constants.RobotConstants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Hopper;
import frc.robot.Subsystems.IntakeShooter;

public class RobotContainer {

  private CommandXboxController mController = new CommandXboxController(RobotConstants.kDriveControllerPort);
  private Drivetrain mDrive = Drivetrain.getInstance();
  private Climber mClimber = Climber.getInstance();
  private IntakeShooter mFuel = IntakeShooter.getInstance();
  private Hopper mHopper = Hopper.getInstance();

  public RobotContainer() {
    configureBindings();
    mDrive.setDefaultCommand(new DefaultDrive(mDrive, mController));
  }

  private void configureBindings() {
    //mController.leftTrigger(0.7, null).onTrue(Commands.run(() -> {mFuel.setShooter(-0.9);}, mFuel));
    //mController.rightTrigger(0.7, null).onTrue(Commands.run(() -> {mFuel.setIntake(-0.5);}, mFuel));
    //mController.leftTrigger().whileTrue(Commands.run(() -> {mFuel.setShooter(-0.9);})).whileFalse(Commands.run(() -> {mFuel.setShooter(0);})); // en donde esta el -0.9 modifica la velocidad del shooter
    //mController.rightTrigger().whileTrue(Commands.run(() -> {mFuel.setIntake(-0.9);})).whileFalse(Commands.run(() -> {mFuel.setIntake(0);})); // en donde esta el -0.8 modifica la velocidad del intake
    // los 0 se encargan de detener los motores, si los modifica los motores ya no se van a detener
    //mController.a().whileTrue(new DropFuel(mFuel));
    //mController.x().whileTrue(new GetFuel(mFuel));
    mController.leftTrigger().whileTrue(new GetFuel(mFuel));
    mController.rightTrigger().whileTrue(new DropFuel(mFuel));
    mController.b().whileTrue(new DropFuelDown(mFuel));
    mController.y().onTrue(new UpRobot(mClimber));
    mController.leftBumper().onTrue(new DownRobot(mClimber));
    mController.povDown().onTrue(new CloseHopper(mHopper));
    mController.povUp().onTrue(new OpenHopper(mHopper));
  }

  public Command getAutonomousCommand() {
    //return Commands.print("No autonomous command configured");
    /*return Commands.sequence(
      new FollowAprilTagAuto(),
      new DropFuel(mFuel).withTimeout(10)
    );*/
    //return new PathPlannerAuto("TestFollowPath");
    return Commands.sequence(
      new MoveBack(mDrive),
      new DropFuel(mFuel).withTimeout(3),
      new OpenHopper(mHopper),
      new AlignEndMoveBack(mDrive),
      new CloseHopper(mHopper)
      //new UpRobot(mClimber)
    );
  }
}
