// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {

    public class DriveConstants{
        public static int kLeftFront = 1;
        public static int kLeftBack = 2;
        public static int kRightFront = 3;
        public static int kRightBack = 4;
        public static MotorType kType = MotorType.kBrushless;
        public static int kCurrentLimit = 50;
        public static double kVelLimit = 0.7;
        public static double kRotLimit = 0.8;
        public static double kTrackWidth = Units.inchesToMeters(28);
        public static double kDriveGear = 1/8.46; // Relacion 8.46:1 (Primer numero entrada, segundo numero salida)
        public static double kWheelDiam = Units.inchesToMeters(6);
        public static double kVelConversionFactor = kDriveGear * 1/60 * Math.PI * kWheelDiam;// RPM -> M/S
        public static double kPosConversionFactor = kDriveGear * Math.PI * kWheelDiam; // Revolucion -> M
        public static double kMaxXVel = 5; // Max Linear vel 5 M/S
        public static double kMaxOVel = 3.14; // Max Rotation vel 3.14 Rad/s o 180 Deg/s
        public static double kP = 0.64899;
        public static double kD = 0.0;
        public static double kS = 0.22879;
        public static double kV = 2.0125;
        public static double kA = 0.45736; 
    }

    public class RobotConstants{
        public static int kDriveControllerPort = 0;
    }

    public class FuelConstants{
        public static int kShooterMotor1 = 5; // Neo 2
        public static int kShooterMotor2 = 6; // Neo 1
        public static int kDirection = 7; // Falcon
        public static MotorType kType = MotorType.kBrushless;
        public static int kCurrentLimit = 40;
        public static double kP = 0.42146;//5.1019;
        public static double KD = 0;
        public static double kS = 0.18314;//0.25283;
        public static double kV = 7.1803;//8.6519;
        public static double kA = 0.55577;//4.9757;
    }

    public class ClimberConstants{
        public static int kMotor = 8;
    }

    public class VisionConstants{
        public final static String kLimelightName = "limelight";
    }
}
