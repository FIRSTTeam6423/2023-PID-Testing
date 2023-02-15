// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AlignToNearestGridTag;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoDriveForwards;
import frc.robot.commands.AutoFollowTrajectory;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.OperateClaw;
import frc.robot.commands.Balance;
import frc.robot.commands.EventFollow;
import frc.robot.commands.OperateDrive;
import frc.robot.subsystems.ClawUtil;
import frc.robot.subsystems.DriveUtil;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...

	private final DriveUtil driveUtil = new DriveUtil();
	private final ClawUtil clawUtil = new ClawUtil();
	private static final PhotonCamera camera = new PhotonCamera("johncam");

	private final OperateDrive operateDrive = new OperateDrive(driveUtil);
	private final OperateClaw operateClaw = new OperateClaw(clawUtil);

	private static XboxController driver;
	private JoystickButton balanceButton, clawButton, testButton;
	// private static XboxController operator;

	/**
	 * Added a new object - JoystickButton
	 * This one is used to Toggle the Climb Arm out and back.
	 */

	public static SendableChooser<Byte> driveType;
	public static SendableChooser<Byte> noobMode;
	public static SendableChooser<String> teamColorChooser;
	public final static Byte arcade = 0;
	public final static Byte tank = 1;
	public final static Byte curvature = 2;

	private SendableChooser<Command> autoChooser = new SendableChooser<>();

	private PathPlannerTrajectory trajectory;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		driver = new XboxController(Constants.XBOX_DRIVER);

		driveType = new SendableChooser<>();
		driveType.setDefaultOption("Arcade", arcade);
		driveType.addOption("Tank", tank);
		driveType.addOption("Curvature", curvature);
		SmartDashboard.putData("Drive Type", driveType);

		trajectory = PathPlanner.loadPath("Test Path", new PathConstraints(Constants.kMaxSpeedMetersPerSecond,
				Constants.kMaxAccelerationInchesPerSecondSquared));

		// Configure the button bindings
		configureButtonBindings();
		configureDefaultCommands();

		// 27 in is a square
		autoChooser.setDefaultOption("Test Trajectory", new AutoFollowTrajectory(driveUtil, trajectory));
		autoChooser.addOption("TestEvent", new EventFollow(driveUtil, trajectory));
		autoChooser.addOption("Drive 132 inches", new AutoDriveForwards(driveUtil, 132));
		autoChooser.addOption("Drive 132 inches Backwards", new AutoDriveForwards(driveUtil, -132));
		autoChooser.addOption("Drive 120 inches", new AutoDriveForwards(driveUtil, 120));
		autoChooser.addOption("Drive 8 Squares", new AutoDriveForwards(driveUtil, 216));
		autoChooser.addOption("Autoturn 5 Seconds", new AutoTurn(driveUtil, 0.5, 5));
		autoChooser.addOption("AutoDrive 8 Seconds", new AutoDrive(driveUtil, .5, 8));
		autoChooser.addOption("testLineToTag",
				new AlignToNearestGridTag(driveUtil, new Translation2d(2, 4.45), Rotation2d.fromDegrees(0), null));
		// new AlignToNearestGridTag(driveUtil, new Translation2d(2, 5),
		// Rotation2d.fromDegrees(0), null));

		SmartDashboard.putData("Autonomous Command", autoChooser);
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		balanceButton = new JoystickButton(driver, Button.kB.value);
		clawButton = new JoystickButton(driver, Button.kX.value);
		testButton = new JoystickButton(driver, Button.kStart.value);
		/**
		 * Actually added code here this time.
		 * First you instantiate your Button (toggleClimb).
		 * Although you use the generic JoystickButton class here
		 * be careful. The second variable in the constructor does all the work.
		 * I used the Button object in this class:
		 * edu.wpi.first.wpilibj.XboxController.Button;
		 * However there are Button classes for PS4 game controllers and more!!!!
		 * Careful what you choose!
		 * 
		 */
		balanceButton.onTrue(new Balance(driveUtil));
		clawButton.onTrue(new InstantCommand(() -> clawUtil.toggleClaw(), clawUtil));
		// testButton.onTrue(new InstantCommand(() -> driveUtil.testSwerve(),
		// driveUtil));
		/**
		 * Could have done this any number of ways, a real command or an instant
		 * command.
		 * I went with InstantCommand, just as an example. It will work. Much more
		 * lightweight
		 * than a full blown Command class given we just want to toggle the state of
		 * something.
		 * 
		 * You will notice that we got rid of the OperateCommand() command class as it
		 * is
		 * not needed in the case of an InstantCommand().
		 * 
		 */
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return autoChooser.getSelected();
	}

	private void configureDefaultCommands() {
		driveUtil.setDefaultCommand(operateDrive);
		clawUtil.setDefaultCommand(operateClaw);
	}

	public static double getDriverLeftXboxX() {
		return driver.getLeftX();
	}

	public static double getDriverLeftXboxY() {
		return driver.getLeftY();
	}

	public static double getDriverRightXboxX() {
		return driver.getRightX();
	}

	public static double getDriverRightXboxY() {
		return driver.getRightY();
	}

	public static double getDriverLeftXboxTrigger() {
		return driver.getLeftTriggerAxis();
	}

	public static double getDriverRightXboxTrigger() {
		return driver.getRightTriggerAxis();
	}

	public static boolean getDriverAButton() {
		return driver.getAButton();
	}

	public static boolean getDriverBButton() {
		return driver.getBButton();
	}

	public static boolean getDriverXButton() {
		return driver.getXButton();
	}

	public static boolean getDriverYButton() {
		return driver.getYButton();
	}

	public static boolean getDriverLeftBumper() {
		return driver.getLeftBumper();
	}

	public static boolean getDriverRightBumper() {
		return driver.getRightBumper();
	}

	public static boolean getDriverLeftStickButton() {
		return driver.getLeftStickButton();
	}

	public static boolean getDriverRightStickButton() {
		return driver.getRightStickButton();
	}

	public static Pose3d getTagPose3dFromId(int id) {
		return Constants.TagPoses[id - 1];
	}

	public static PhotonTrackedTarget getNearestCameraTarget() {
		PhotonPipelineResult result = camera.getLatestResult();
		if (result.hasTargets()) {
			return result.getBestTarget();
		}
		return null;
	}

	public static List<PhotonTrackedTarget> getAllCameraTargets() {
		PhotonPipelineResult result = camera.getLatestResult();
		if (result.hasTargets()) {
			return result.getTargets();
		} else {
			return Collections.<PhotonTrackedTarget>emptyList();
		}
	}

	public static Pose3d getPose3dOfNearestCameraTarget() {
		PhotonPipelineResult result = camera.getLatestResult();
		if (result.hasTargets()) {
			PhotonTrackedTarget target = result.getBestTarget();
			Pose3d tagPose = getTagPose3dFromId(target.getFiducialId());
			return tagPose;
		}
		return null;
	}

	public static Pose2d getFieldPosed2dFromNearestCameraTarget() {
		PhotonPipelineResult result = camera.getLatestResult();
		if (result.hasTargets()) {
			PhotonTrackedTarget target = result.getBestTarget();
			Pose3d tagPose = getTagPose3dFromId(target.getFiducialId());
			Pose3d pos = PhotonUtils.estimateFieldToRobotAprilTag(
					target.getBestCameraToTarget(),
					tagPose, Constants.CAMERA_TO_ROBOT);
			return new Pose2d(
					pos.getX(),
					pos.getY(),
					new Rotation2d(pos.getRotation().getZ()));
		}
		DriverStation.reportWarning("Could not get Pose2d from camera target: no targets found.", false);
		return null;
	}
}