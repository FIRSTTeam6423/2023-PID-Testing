// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
        public static final double TICKS_PER_METER = 103308.83;

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
        public static final double kaVoltSecondsSquaredPerMeters = 0;//0.38619; // changes from meters to inches

        //public static final double kPDrivePos = 5.4454;
        //public static final double kDDrivePos = 0.77378;

        public static final double kPDriveVel = 0.074925;

        public static final double kTrackwidthInches = 0.57912; //22.8 inches
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthInches);
        
        public static final double kMaxSpeedMetersPerSecond = 0.5; //1783.16799018 inches per second maximum free speed
        public static final double kMaxAccelerationInchesPerSecondSquared = 0.5; //Magic


        public static final double BEAM_BALANACED_DRIVE_KP = 0.006; // P (Proportional) constant of a PID loop
        public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
        public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1.6;
        /**
        public static final double kMaxSpeedMetersPerSecond = 0.1; //1783.16799018 inches per second maximum free speed
        public static final double kMaxAccelerationInchesPerSecondSquared = 0.1; //Magic

        public static final double kRamseteB = 2; //convergence
        public static final double kRamseteZeta = 0.7; // damping


        */
        public static final double APRIL1_X = 15.51;
        public static final double APRIL1_Y = 1.07;
        public static final double APRIL1_ROT = 180;
        public static final double APRIL2_X = 15.51;
        public static final double APRIL2_Y = 2.75;
        public static final double APRIL2_ROT = 180;
        public static final double APRIL3_X = 15.51;
        public static final double APRIL3_Y = 3.74;
        public static final double APRIL3_ROT = 180;
        public static final double APRIL4_X = 16.18;
        public static final double APRIL4_Y = 4.42;
        public static final double APRIL4_ROT = 180;
        public static final double APRIL5_X = 0.36;
        public static final double APRIL5_Y = 4.42;
        public static final double APRIL5_ROT = 0;
        public static final double APRIL6_X = 1.03;
        public static final double APRIL6_Y = 4.42;
        public static final double APRIL6_ROT = 0;
        public static final double APRIL7_X = 1.03;
        public static final double APRIL7_Y = 3.74;
        public static final double APRIL7_ROT = 0;
        public static final double APRIL8_X = 1.03;
        public static final double APRIL8_Y = 1.07;
        public static final double APRIL8_ROT = 0;
        public static final double GRID_TAG_HEIGHT = 0.46; // Tags 1-3 (red) & 6-8 (blue)
        public static final double SUB_TAG_HEIGHT = 0.67; //Tags 4-5
        public static final Pose3d[] TagPoses = {
            new Pose3d(APRIL1_X,APRIL1_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL1_ROT))),
            new Pose3d(APRIL2_X,APRIL2_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL2_ROT))),
            new Pose3d(APRIL3_X,APRIL3_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL3_ROT))),
            new Pose3d(APRIL4_X,APRIL4_Y,SUB_TAG_HEIGHT,  new Rotation3d(0, 0, Math.toRadians(APRIL4_ROT))),
            new Pose3d(APRIL5_X,APRIL5_Y,SUB_TAG_HEIGHT,  new Rotation3d(0, 0, Math.toRadians(APRIL5_ROT))),
            new Pose3d(APRIL6_X,APRIL6_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL6_ROT))),
            new Pose3d(APRIL7_X,APRIL7_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL7_ROT))),
            new Pose3d(APRIL8_X,APRIL8_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL8_ROT))),
            };

        public static final Transform3d CAMERA_TO_ROBOT=new Transform3d();
        
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