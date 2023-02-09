// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFollowTrajectory extends SequentialCommandGroup {
  /** Creates a new AutoFollowTrajectory. */
  public AutoFollowTrajectory(DriveUtil driveUtil, PathPlannerTrajectory trajectory) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> {
          driveUtil.resetOdometry(trajectory.getInitialPose());
        }),
        new PPRamseteCommand(
            trajectory,
            driveUtil::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeters, .1),//Constants.kaVoltSecondsSquaredPerMeters),
            Constants.kDriveKinematics, // DifferentialDriveKinematics
            driveUtil::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(Constants.DRIVER_P, Constants.DRIVER_I, Constants.DRIVER_D), // Left controller. Tune these values for your robot. Leaving them 0 will only
                                        // use feedforwards.
            new PIDController(Constants.DRIVER_P, Constants.DRIVER_I, Constants.DRIVER_D), // Right controller (usually the same values as left controller)
            driveUtil::tankDriveVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            driveUtil // Requires this drive subsystem
        ));
  }
}
