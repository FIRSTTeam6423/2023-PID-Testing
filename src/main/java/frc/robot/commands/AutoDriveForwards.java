// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveUtil;

public class AutoDriveForwards extends CommandBase {
  /** Creates a new AutoTurn. */
  private DriveUtil driveUtil;
  private boolean done;
  private double encoderSetpoint;
  private double inches;
  public AutoDriveForwards(DriveUtil du, double inches) {
    this.driveUtil = du;
    this.inches = inches * Constants.TICKS_PER_INCH;
    addRequirements(this.driveUtil);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    driveUtil.resetEncoders();
    encoderSetpoint = driveUtil.getPosition() + inches;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (!driveUtil.getMoving() && driveUtil.getPosition() > encoderSetpoint - Constants.DRIVER_DEADBAND 
    //&& driveUtil.getPosition() < encoderSetpoint + Constants.DRIVER_DEADBAND ){
    driveUtil.driveToSetpoint(driveUtil.getPosition(), encoderSetpoint);

    if (driveUtil.atCurrentPIDSetpoint())
    {
        driveUtil.stopDistance();
        done = true;
        driveUtil.stopDistance();
        System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
        return;
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveUtil.stopDistance();
    System.out.println("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
