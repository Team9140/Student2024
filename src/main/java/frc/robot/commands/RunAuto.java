package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;

public class RunAuto extends Command {
    private final FollowPath[] paths;
    private final ArrayList<Command> blockedEvents;
    private final ArrayList<ArrayList<Command>> parallelEvents;
    private Drivetrain m_drive;
    private int currentPath;

    public RunAuto(String name, Drivetrain drivetrain, int numPaths, boolean mirror) {
        this.m_drive = drivetrain;
        this.paths = new FollowPath[numPaths];
        this.blockedEvents = new ArrayList<Command>(numPaths + 1);

        for (int i = 0; i < numPaths + 1; i++) {
            this.blockedEvents.add(null);
        }

        this.parallelEvents = new ArrayList<>();

        for (int i = 0; i < numPaths; i++) {
            this.parallelEvents.add(new ArrayList<Command>());
        }

        for (int i = 0; i < numPaths; i++) {
            this.paths[i] = new FollowPath(String.format("%s.%d", name, i + 1), this.m_drive, mirror);
        }

        Field2d field = new Field2d();

        for (FollowPath path : this.paths) {
            List<Pose2d> traj = field.getObject("traj").getPoses();
            traj.add(path.getInitialPose());
            field.getObject("traj").setPoses(traj);

            List<Pose2d> trajPoses = field.getObject("trajPoses").getPoses();
            trajPoses.addAll(path.getPoses());
            field.getObject("trajPoses").setPoses(trajPoses);

            SmartDashboard.putData(field);
        }
        List<Pose2d> traj = field.getObject("traj").getPoses();
        traj.add(this.paths[this.paths.length - 1].getFinalPose());
        field.getObject("traj").setPoses(traj);
    }

    public void setBlockedEvent(Command event, int position) {
        this.blockedEvents.set(position, event);
    }

    public void scheduleParallelEvent(Command event, int stopIndex, double offset) {
        if (offset >= 0) {
            this.parallelEvents.get(stopIndex).add(new WaitCommand(offset).andThen(event));
        } else {
            this.parallelEvents.get(stopIndex - 1).add(new WaitCommand(paths[stopIndex - 1].getTotalTime() + offset).andThen(event));
        }
    }

    private Command getPathCommand(int pathIndex) {
        Command blockedEvent = this.blockedEvents.get(pathIndex);

        if (pathIndex == this.paths.length) {
            return blockedEvent != null ? blockedEvent : new WaitCommand(0.0);
        }

        ParallelCommandGroup path = new ParallelCommandGroup(this.paths[pathIndex]);

        for (Command parallelEvent : this.parallelEvents.get(pathIndex)) {
            path.addCommands(parallelEvent);
        }

        if (blockedEvent != null) {
            return blockedEvent.andThen(path);
        }

        return path;
    }

    @Override
    public final void initialize() {
        currentPath = 0;

        if (this.paths.length == 0) {
            return;
        }

        this.getPathCommand(0).initialize();

        this.m_drive.seedFieldRelative(this.paths[0].getInitialPose());

        SmartDashboard.putString("starting position", this.paths[0].getInitialPose().toString());
        SmartDashboard.putString("desired ending position", this.paths[this.paths.length - 1].getFinalPose().toString());
    }

    @Override
    public final void execute() {
        if (this.paths.length == 0) {
            return;
        }

        Command currentCommand = this.getPathCommand(currentPath);

        currentCommand.execute();
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            currentPath++;
            if (currentPath <= this.paths.length) {
                this.getPathCommand(currentPath).initialize();
            }
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (interrupted
                && this.paths.length != 0
                && this.currentPath > -1
                && this.currentPath <= this.paths.length) {
            this.getPathCommand(this.currentPath).end(true);
        }
        this.currentPath = -1;
    }

    @Override
    public final boolean isFinished() {
        return this.currentPath == this.paths.length + 1;
    }
}