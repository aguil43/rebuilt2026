// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private static Drivetrain mDrivetrain;

  private SparkMax mLeftBack;
  private SparkMax mLeftFront;
  private SparkMax mRightBack;
  private SparkMax mRightFront;

  private RelativeEncoder mLeftBackMotorEncoder;
  private RelativeEncoder mLeftFrontMotorEncoder;
  private RelativeEncoder mRightBackMotorEncoder;
  private RelativeEncoder mRightFrontMotorEncoder;

  private DifferentialDrive mDrive;

  private AHRS mGyro;
  private DifferentialDriveKinematics mKinematics;
  private DifferentialDrivePoseEstimator mOdometry;
  private Field2d mOdomeField;
  private Field2d mVisionField;
  private PIDController mLeftPID;
  private PIDController mRightPID;
  private SimpleMotorFeedforward mFFLeft;
  private SimpleMotorFeedforward mFFRight;
  private RobotConfig config;

  private Vector<N2> relems = VecBuilder.fill(1, 2);
  private Vector<N3> quelems = VecBuilder.fill(0.125, 0.125, 3);


  public Drivetrain() {
    mLeftBack = new SparkMax(DriveConstants.kLeftBack, DriveConstants.kType);
    mLeftFront = new SparkMax(DriveConstants.kLeftFront, DriveConstants.kType);
    mRightBack = new SparkMax(DriveConstants.kRightBack, DriveConstants.kType);
    mRightFront = new SparkMax(DriveConstants.kRightFront, DriveConstants.kType);

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leftMasterConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightMasterConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(DriveConstants.kCurrentLimit).idleMode(IdleMode.kCoast)
    .encoder.velocityConversionFactor(DriveConstants.kVelConversionFactor).positionConversionFactor(DriveConstants.kPosConversionFactor);
    leftMasterConfig.apply(globalConfig).inverted(true);
    rightMasterConfig.apply(globalConfig);
    leftFollowerConfig.apply(globalConfig).follow(mLeftBack);
    rightFollowerConfig.apply(globalConfig).follow(mRightBack);

    mLeftBack.configure(leftMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mLeftFront.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mRightBack.configure(rightMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mRightFront.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mDrive = new DifferentialDrive(mLeftBack, mRightBack);
    mDrive.setSafetyEnabled(false);

    mLeftBackMotorEncoder = mLeftBack.getEncoder();
    mLeftFrontMotorEncoder = mLeftFront.getEncoder();
    mRightBackMotorEncoder = mRightBack.getEncoder();
    mRightFrontMotorEncoder = mRightFront.getEncoder();

    mLeftPID = new PIDController(DriveConstants.kP, 0.0, DriveConstants.kD);
    mRightPID = new PIDController(DriveConstants.kP, 0.0, DriveConstants.kD);

    mFFLeft = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);
    mFFRight = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

    mGyro = new AHRS(NavXComType.kMXP_SPI);
    mKinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);
    mOdometry = new DifferentialDrivePoseEstimator(mKinematics, mGyro.getRotation2d(), getLeftAvgPose(), getRightAvgPose(), new Pose2d(1.768, 6.0, new Rotation2d()));
    // 0 degrees / radians represents the robot angle when the robot is facing directly toward your opponent’s alliance station.
    // As your robot turns to the left, your gyroscope angle should increase.
    mOdomeField = new Field2d();
    mVisionField = new Field2d();

    // TODO: Consutruir el autoBuilder de pathplanner aqui
    try{
      config = RobotConfig.fromGUISettings();
    }catch(Exception e){
      System.out.println("Error al cargar la configuracion: " + e.getMessage());
    }

    try{
      AutoBuilder.configure(
      this::getPose,
      this::resetPose, 
      this::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> testDrive(speeds), 
      new PPLTVController(0.02, 0.4), 
      config,
      () -> {
        var alliance =  DriverStation.getAlliance();
        if(alliance.isPresent()){
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this);
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner and configure AutoBuildes", e.getStackTrace());
    }
  }

  public void teleopDrive(double xVel, double zVel){
    mDrive.arcadeDrive(xVel * DriveConstants.kVelLimit, zVel * DriveConstants.kRotLimit, true);
  }

  public void autoDrive(double leftVel, double rightVel){
    mDrive.tankDrive(leftVel, rightVel, true);
  }

  public void setSpeeds(double leftVolt, double rightVolt){
    mLeftBack.setVoltage(leftVolt);
    mRightBack.setVoltage(rightVolt);
  }

  public void testTeleopDrive(double linear, double rot){
    ChassisSpeeds speeds = new ChassisSpeeds(linear, 0, rot);
    testDrive(speeds);
  }

  public void testDrive(ChassisSpeeds speeds){
    DifferentialDriveWheelSpeeds wheelSpeeds = mKinematics.toWheelSpeeds(speeds);
    double leftVolt = 0;
    double rightVolt = 0;

    double leftPID = mLeftPID.calculate(mLeftBackMotorEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond);
    double rightPID = mRightPID.calculate(mRightBackMotorEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond);

    double leftFF = mFFLeft.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightFF = mFFRight.calculate(wheelSpeeds.rightMetersPerSecond);

    leftVolt = leftFF + leftPID;
    rightVolt = rightFF + rightPID;
    
    setSpeeds(leftVolt, rightVolt);
  }

  public double getLeftAvgPose(){
    double avg = (mLeftBackMotorEncoder.getPosition() + mLeftFrontMotorEncoder.getPosition())/2;
    return avg;
  }

  public double getRightAvgPose(){
    double avg = (mRightBackMotorEncoder.getPosition() + mRightFrontMotorEncoder.getPosition())/2;
    return avg;
  }

  public double getLeftAvgVel(){
    double avg = (mLeftBackMotorEncoder.getVelocity() + mLeftFrontMotorEncoder.getVelocity())/2;
    return avg;
    //return mLeftBackMotorEncoder.getVelocity();
  }

  public double getRightAvgVel(){
    double avg = (mRightBackMotorEncoder.getVelocity() + mRightFrontMotorEncoder.getVelocity())/2;
    return avg;
    //return mRightBackMotorEncoder.getVelocity();
  }

  public Pose2d getPose(){
    return mOdometry.getEstimatedPosition();
  }

  public void resetPose(Pose2d newPose){
    mOdometry.resetPose(newPose);
    mLeftBackMotorEncoder.setPosition(0);
    mRightBackMotorEncoder.setPosition(0);
    mGyro.reset();
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return mKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftAvgVel(), getRightAvgVel()));
  }

  public void updateWithVision(){
    double[] botpose = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName).getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    Pose2d visionPose = new Pose2d(0, 0, Rotation2d.kZero);

    if(botpose.length > 0 && NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName).getEntry("tv").getDouble(0) > 0){
      visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));

      //double timestamp = Timer.getFPGATimestamp() - (botpose[6] / 1000.0);
      //mOdometry.addVisionMeasurement(visionPose, timestamp);
      mVisionField.setRobotPose(visionPose);
    }
    //SmartDashboard.putData(mVisionField);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    mOdometry.update(mGyro.getRotation2d(), getLeftAvgPose(), getRightAvgPose());
    //updateWithVision();
    mOdomeField.setRobotPose(mOdometry.getEstimatedPosition());
    SmartDashboard.putData(mOdomeField);

    SmartDashboard.putNumber("Output left", mLeftBack.getAppliedOutput());
    SmartDashboard.putNumber("Output right", mRightBack.getAppliedOutput());

    SmartDashboard.putNumber("Angle", mGyro.getAngle());

    SmartDashboard.putNumber("Left vel", getLeftAvgVel());
    SmartDashboard.putNumber("Right vel", getRightAvgVel());
    SmartDashboard.putNumber("Left Pose", getLeftAvgPose());
    SmartDashboard.putNumber("Right Pose", getRightAvgPose());
  }

  public static Drivetrain getInstance(){
    if(mDrivetrain == null){
      mDrivetrain = new Drivetrain();
    }
    return mDrivetrain;
  }
}
