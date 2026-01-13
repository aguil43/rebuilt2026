// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class Constants {

    public class DriveConstants{
        public static int kLeftFront = 1;
        public static int kLeftBack = 2;
        public static int kRightFront = 3;
        public static int kRightBack = 4;
        public static MotorType kType = MotorType.kBrushed;
        public static int kCurrentLimit = 50;
        public static double kVelLimit = 0.7;
        public static double kRotLimit = 0.8;
    }

    public class RobotConstants{
        public static int kDriveControllerPort = 0;
    }

    public class FuelConstants{
        public static int kShooter = 5;
        public static int kDirection = 6;
        public static MotorType kType = MotorType.kBrushed;
    }

}
