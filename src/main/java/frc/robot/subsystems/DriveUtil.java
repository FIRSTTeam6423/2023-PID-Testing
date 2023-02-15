// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
//import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveUtil extends SubsystemBase {
    private CANSparkMax leftPrimary, leftSecondary, rightPrimary, rightSecondary, testlinear, testrotate;

    private RelativeEncoder leftPrimaryEncoder, leftSecondaryEncoder, rightPrimaryEncoder, rightSecondaryEncoder;

    private PIDController linearPIDController; 
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final DifferentialDriveOdometry odometry;

    public double setpoint;
    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("johncam");

    final double CAMERA_HEIGHT = 0.8128;
    final double TARGET_HEIGHT = 1.2446;
    final double CAMERA_PITCH_RADIANS = 0;
    final double GOAL_RANGE_METERS = 0.3;

    double yaw;
    double range;
    double linearSpeed;
    double rotationSpeed;
    // Drive controller
    private DifferentialDrive differentialDrive;

    public DriveUtil() {
        leftPrimary = new CANSparkMax(Constants.LEFT_PRIMARY, MotorType.kBrushless);
        leftSecondary = new CANSparkMax(Constants.LEFT_SECONDARY, MotorType.kBrushless);
        rightPrimary = new CANSparkMax(Constants.RIGHT_PRIMARY, MotorType.kBrushless);
        rightSecondary = new CANSparkMax(Constants.RIGHT_SECONDARY, MotorType.kBrushless);
        // testlinear = new CANSparkMax(9, MotorType.kBrushless);
        // testrotate = new CANSparkMax(10, MotorType.kBrushless);
        // rightPrimary.setInverted(false);

        gyro = new AHRS();
        gyro.reset();
        gyro.calibrate();

        //gyro.reset();
        setpoint = 0;

        // Set secondaries to follow primaries
        leftSecondary.follow(leftPrimary);
        rightSecondary.follow(rightPrimary);
        camera.setLED(VisionLEDMode.kOn);
        // Initialize DifferentialDrive controller
        differentialDrive = new DifferentialDrive(leftPrimary, rightPrimary);

        leftPrimaryEncoder = leftPrimary.getEncoder();
        leftSecondaryEncoder = leftSecondary.getEncoder();
        rightPrimaryEncoder = rightPrimary.getEncoder();
        rightSecondaryEncoder = rightSecondary.getEncoder();

        linearPIDController = new PIDController(Constants.DRIVER_P, Constants.DRIVER_I, Constants.DRIVER_D);
        
        leftPrimaryEncoder.setPositionConversionFactor(4096);
        leftSecondaryEncoder.setPositionConversionFactor(4096);
        rightPrimaryEncoder.setPositionConversionFactor(4096);
        rightSecondaryEncoder.setPositionConversionFactor(4096);

        leftPrimary.setInverted(true);
        // leftPrimaryEncoder.setInverted(true);
        // rightPrimaryEncoder.setInverted(true);
        // leftSecondaryEncoder.setInverted(true);
        // rightSecondaryEncoder.setInverted(true);

        leftPrimaryEncoder.setPosition(0);
        rightPrimaryEncoder.setPosition(0);
        leftSecondaryEncoder.setPosition(0);
        rightSecondaryEncoder.setPosition(0);

        leftPrimary.setIdleMode(IdleMode.kCoast);
        rightPrimary.setIdleMode(IdleMode.kCoast);
        leftSecondary.setIdleMode(IdleMode.kCoast);
        rightSecondary.setIdleMode(IdleMode.kCoast);
        
        odometry = 
            new DifferentialDriveOdometry(
                gyro.getRotation2d(), 
                leftPrimaryEncoder.getPosition() / Constants.TICKS_PER_METER, 
                rightPrimaryEncoder.getPosition() / Constants.TICKS_PER_METER
            );


        linearPIDController.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(
            gyro.getRotation2d(), leftPrimaryEncoder.getPosition(), rightPrimaryEncoder.getPosition(), pose);
        System.out.println("ayo we reset the GYROOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
    } 

    public void zeroHeading(double angle) {
        gyro.reset();
    }

    public double getHeading () {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return gyro.getRate(); //might need to negate?
    }

    
    /**
     * Drive the robot based on the driveMode class parameter.
     * If in TANK mode, use leftX and rightX values.
     * If in ARCADE mode, use rightX and rightY values.
     * 
     * The DifferentialDrive class will square inputs for us.
     * Squaring inputs results in less sensitive inputs.
     * 
     * @param leftX  the left controller's X (forward-backward) value
     * @param leftY  the left controller's Y (left-right) value
     * @param rightX the right controller's X (forward-backward) value
     * @param rightY the right controller's Y (left-right) value
     */
    public void driveRobot() {
        // arcade drive
        if (RobotContainer.getDriverBButton()) {
            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                range = PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT,
                        TARGET_HEIGHT,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(target.getPitch()));
                // range = target.getArea();
                // if (range > 0.3){
                // range = 1;
                // }
                yaw = target.getYaw();
                if (Math.abs(yaw) < 1.5) {
                    yaw = 0;
                }
                linearSpeed = -25 * linearPIDController.calculate(range, GOAL_RANGE_METERS);
                if (Math.abs(yaw) > 11) {
                    rotationSpeed = 4 * Math.sin(0.2 * Math.PI * yaw / 180);
                } else {
                    rotationSpeed = 0.155 * Math.sin(9 * Math.PI * yaw / 180);
                }
                // Forward speed and rotation speed reversed for god knows why.
                // Maybe motor id problems
                differentialDrive.arcadeDrive(MathUtil.clamp(rotationSpeed, -0.35, 0.35), linearSpeed); // MathUtil.clamp(linearSpeed,
                                                                                                        // -0.5, 0.5)
            } else {
                differentialDrive.arcadeDrive(0, 0);
            }
        } else {
            if (RobotContainer.driveType.getSelected().equals(RobotContainer.arcade)) {
                // If we're in ARCADE mode, use arcadeDrive

                differentialDrive.arcadeDrive(RobotContainer.getDriverRightXboxX(),
                        -RobotContainer.getDriverRightXboxY() / 1.5);
            } else if (RobotContainer.driveType.getSelected().equals(RobotContainer.tank)) {
                // If we're in TANK mode, use tankDrive
                differentialDrive.tankDrive(-RobotContainer.getDriverLeftXboxY() / 2,
                        RobotContainer.getDriverRightXboxY() / 2);

            } else {
                // If we are in CURVATURE mode, use the curvature mode
                double rotation = RobotContainer.getDriverLeftXboxX();
                boolean isNegative = rotation < 0;

                rotation *= rotation;
                if (isNegative) {
                    rotation *= -1;
                }
                rotation *= 0.75;

                differentialDrive.curvatureDrive(rotation,
                        (-RobotContainer.getDriverLeftXboxTrigger() + RobotContainer.getDriverRightXboxTrigger()) / 2,
                        true);
            }
        }

    }

    // public void testSwerve() {
    //     testlinear.setVoltage(0.1);
    //     testrotate.setVoltage(0.1);
    // }

    // public void stopSwerve() {
    //     testlinear.setVoltage(0);
    //     testrotate.setVoltage(0);
    // }

    public void setPIDPositionTolerance(double tolerance) {
        linearPIDController.setTolerance(tolerance);
    }

    public void tankDrive(double left, double right) {
        differentialDrive.tankDrive(left, right);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftPrimary.setVoltage(leftVolts);
        rightPrimary.setVoltage(rightVolts); //probably is inverted rn
        differentialDrive.feed();
        
        // SmartDashboard.putNumber("target left volts", leftVolts);
        // SmartDashboard.putNumber("target right volts", rightVolts);
        // System.out.println("target left volts: "+leftVolts);
        // System.out.println("target right volts: "+rightVolts);
    }

    public void setLinearPIDSetpoint(double setpoint) {
        linearPIDController.setSetpoint(setpoint);
    }

    public boolean driveToSetpoint(double currentpos, double setpoint) {
        differentialDrive.arcadeDrive(0, -MathUtil.clamp(linearPIDController.calculate(currentpos), -0.7, 0.7));
        if (linearPIDController.atSetpoint())
            return true;
        return false;
    }

    public void stopDistance() {
        differentialDrive.tankDrive(0, 0);
    }

    public double getPosition() {
        double sensorPosition = (leftPrimaryEncoder.getPosition() - rightPrimaryEncoder.getPosition()) / 2;

        return sensorPosition;
    }

    public boolean getMoving() {
        return leftPrimary.get() > 0.1 && rightSecondary.get() > 0.1;
    }

    public double getPitch() {
        return gyro.getRoll();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftPrimaryEncoder.getVelocity(),rightPrimaryEncoder.getVelocity());
    }

    

    public void resetEncoders(){
        leftPrimaryEncoder.setPosition(0);
        rightPrimaryEncoder.setPosition(0);
        leftSecondaryEncoder.setPosition(0);
        rightSecondaryEncoder.setPosition(0);
    }

    public void drive(double leftPercentPower, double rightPercentPower) {
        leftPrimary.set(leftPercentPower);
        leftSecondary.set(leftPercentPower);
        rightPrimary.set(rightPercentPower);
        rightSecondary.set(rightPercentPower);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** This is normally where we send important values to the SmartDashboard */
        odometry.update(
            gyro.getRotation2d(),
            leftPrimaryEncoder.getPosition() / Constants.TICKS_PER_METER,
            rightPrimaryEncoder.getPosition() / Constants.TICKS_PER_METER
        );
        Pose2d curOdometryPose =  odometry.getPoseMeters();
        SmartDashboard.putNumber("odometry pos x", leftPrimaryEncoder.getPosition()/Constants.TICKS_PER_METER);//curOdometryPose.getX());
        SmartDashboard.putNumber("odometry pos y", curOdometryPose.getY());
        SmartDashboard.putNumber("odometry angle", curOdometryPose.getRotation().getDegrees());
        //SmartDashboard.putString("Drive Type   ::  ", RobotContainer.driveType.getSelected().toString());
        //SmartDashboard.putString("Yaw   ::  ", Double.toString(yaw));
       
        //SmartDashboard.putNumber("encoder  ::  ", getPosition());
        SmartDashboard.putNumber("encoder left", leftPrimaryEncoder.getPosition());
        SmartDashboard.putNumber("encoder right", rightPrimaryEncoder.getPosition());

        SmartDashboard.putNumber("gyro", gyro.getYaw());//gyro.getRotation2d().getDegrees());
        //SmartDashboard.putString("Range  ::  ", Double.toString(range));
        //SmartDashboard.putString("lSpeed  ::  ", Double.toString(linearSpeed));
    }
}