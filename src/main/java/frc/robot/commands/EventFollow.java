package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveUtil;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;

public class EventFollow extends SequentialCommandGroup {
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    
    public EventFollow(DriveUtil driveUtil, PathPlannerTrajectory trajectory){
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Balance", new PrintCommand("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII"));

        // FollowPathWithEvents command = new FollowPathWithEvents(
        //     new AutoFollowTrajectory(driveUtil, trajectory),
        //     trajectory.getMarkers(),
        //     eventMap
        // );
        
        addCommands(
            // new InstantCommand(() -> {
            //     driveUtil.resetOdometry(trajectory.getInitialPose());
            // }),
            new FollowPathWithEvents(
                new AutoFollowTrajectory(driveUtil, trajectory),
                trajectory.getMarkers(),
                eventMap
            )
        );
    }
}
