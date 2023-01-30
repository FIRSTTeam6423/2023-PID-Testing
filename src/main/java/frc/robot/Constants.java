// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        /** 
         * DriveUtil Constants         */
        public static final int LEFT_PRIMARY = 2;
        public static final int LEFT_SECONDARY = 3;
        public static final int RIGHT_PRIMARY = 4;
        public static final int RIGHT_SECONDARY = 5;

        public static final double AUTO_TURN_SPEED = 0.4;
        public static final double AUTO_TURN_SPEED_DAMPENING = 0.5;
        public static final double AUTO_TURN_SLOWDOWN_RANGE = 30;

        public static final double TICKS_PER_INCH = 2624.04427; //26.412;//52.825;//2267.4512;
        public static final double AUTO_DRIVE_SPEED = 0.4;
        public static final double AUTO_DRIVE_SPEED_DAMPENING = 0.5;
        public static final double AUTO_DRIVE_SLOWDOWN_RANGE = 12;

        public static final double DRIVER_P = 0.0000065;
        public static final double DRIVER_I = 0.00000035;
        public static final double DRIVER_D = 0.0;
        public static final double DRIVER_F = 0.0;
        public static final double DRIVER_DEADBAND = 0;

        public static final double ksVolts = 0.00721745;
        public static final double kvVoltSecondsPerMeters = 2.8097; // 0.069437 in inches
        public static final double kaVoltSecondsSquaredPerMeters = 0.38619; // changes from meters to inches

        //public static final double kPDrivePos = 5.4454;
        //public static final double kDDrivePos = 0.77378;

        public static final double kPDriveVel = 0.074925;

        public static final double kTrackwidthInches = 0.57912; //22.8 inches
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthInches);
        
        public static final double kMaxSpeedMetersPerSecond = 3; //1783.16799018 inches per second maximum free speed
        public static final double kMaxAccelerationInchesPerSecondSquared = 1; //Magic

        public static final double kRamseteB = 2; //convergence
        public static final double kRamseteZeta = 0.7; // damping



        
                /**
         * CargoUtil Constants
         */
        public static final int BALL_MAGNET = 6;
        public static final int LOW_INDEXER = 7;
        public static final int HIGH_INDEXER = 8;
        public static final int SHOOTER = 9;
        public static final int UPPER_LIMIT_SWTICH = 0;
        public static final int LOWER_LIMIT_SWTICH = 1;

        public static final double BALL_MAGNET_OUTPUT = 0.4;
        public static final double INDEXER_OUTPUT = 1;
        public static final double SHOOTER_RPM = 1000.0;
        public static final double SHOOTER_VALUE = 0.75;
        public static final double SHOOTER_RPM_DEADBAND = 100.0;
        public static final double SHOOT_TIME = 0.5;

        public static final int ARCADE_LEFT_DAMPENING = 60;
        public static final int ARCADE_RIGHT_DAMPENING = 60;

        public static final Integer BALL_DISTANCE = 15;

        /**
         * Controller Input Device Mapping
         */
        //public static final int LEFT_STICK = 2;
        //public static final int RIGHT_STICK = 1;
        public static final int XBOX_DRIVER = 0;
        public static final int XBOX_OPERATOR = 1;

        /**
         * Controller Button Mapping
         */
        public static final int kLeftXAxisNum = 0;
        public static final int kLeftYAxisNum = 1;
        public static final int kRightXAxisNum = 2;
        public static final int kRightYAxisNum = 3;

        public static final int kXButtonNum = 1;
        public static final int kAButtonNum = 2;
        public static final int kBButtonNum = 3;
        public static final int kYButtonNum = 4;
        public static final int kLeftBumperNum = 5;
        public static final int kRightBumperNum = 6;
        public static final int kLeftTriggerNum = 7;
        public static final int kRightTriggerNum = 8;
        public static final int kBackButtonNum = 9;
        public static final int kStartButtonNum = 10;
        public static final int kLeftStickButtonNum = 11;
        public static final int kRightStickButtonNum = 12;

}