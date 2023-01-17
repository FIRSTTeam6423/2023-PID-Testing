// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDance extends SequentialCommandGroup {
  /** Creates a new AutoDance. */
  public AutoDance(DriveUtil du) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new AutoTurn(du, -1, .5),
      // new AutoDrive(du, 1, .5),
      // new AutoTurn(du, .7, 1.5),
      // new AutoDrive(du, -.7, 1.5),
      new AutoTurn(du, .5, 2),
      new AutoDrive(du, -0.5, 0.2),
      new AutoTurn(du, 0.3, 1),
      new AutoTurn(du, 0.2, 5),
      new AutoTurn(du, -0.22, 6)
    );
  }
  
}
