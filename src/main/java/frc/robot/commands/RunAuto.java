package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.ChoreoTrajectory;
import frc.robot.lib.EventMarker;
import frc.robot.subsystems.Drivetrain;

import java.util.*;
import java.util.function.Supplier;

public class RunAuto extends Command {
    private final FollowPath[] paths;
    private final ArrayList<Command> blockedEvents;
    private final ArrayList<ArrayList<Pair<Double, Command>>> parallelEvents;
    private final Command[] finalizedPaths;
    private Drivetrain m_drive;
    private int currentPath;
    private TreeMap<String, Supplier<Command>> namedEvents;

    public RunAuto(String name, Drivetrain drivetrain, int numPaths, boolean mirror, Field2d field) {
        this(name, drivetrain, numPaths, mirror);

        ArrayList<Pose2d> traj = new ArrayList<>();
        ArrayList<Pose2d> trajPoses = new ArrayList<>();

        for (FollowPath path : this.paths) {
            traj.add(path.getInitialPose());

            trajPoses.addAll(path.getPoses());
        }

        traj.add(this.paths[this.paths.length - 1].getFinalPose());
        field.getObject("traj").setPoses(traj);
        field.getObject("trajPoses").setPoses(trajPoses);
    }

    public RunAuto(String name, Drivetrain drivetrain, int numPaths, boolean mirror) {
        this.m_drive = drivetrain;
        this.namedEvents = new TreeMap<>();
        this.paths = new FollowPath[numPaths];
        this.blockedEvents = new ArrayList<>(numPaths + 1);
        this.finalizedPaths = new Command[numPaths + 1];

        for (int i = 0; i < numPaths + 1; i++) {
            this.blockedEvents.add(null);
        }

        this.parallelEvents = new ArrayList<>();

        for (int i = 0; i < numPaths; i++) {
            this.parallelEvents.add(new ArrayList<>());
        }

        for (int i = 0; i < numPaths; i++) {
            this.paths[i] = new FollowPath(String.format("%s.%d", name, i + 1), this.m_drive, mirror);
        }
    }

    public void setBlockedEvent(Command event, int position) {
        this.blockedEvents.set(position, event);
    }

    public void setNamedEvent(String name, Supplier<Command> eventSupplier) {
        this.namedEvents.put(name, eventSupplier);
    }
    
    public void parseEventMarkers() {
        for (int i = 0; i < this.paths.length; i++) {
            ChoreoTrajectory trajectory = this.paths[i].getTrajectory();
            List<EventMarker> eventMarkers = trajectory.getEventMarkers();
            for (EventMarker marker : eventMarkers) {
                Supplier<Command> command = this.namedEvents.get(marker.getCommand().getData().getName());
                if (command != null) {
                    this.scheduleParallelEvent(command.get(), i, marker.getTimestamp());
                }
            }
        }
    }

    public void scheduleParallelEvent(Command event, int stopIndex, double offset) {
        if (offset >= 0) {
            if (offset > paths[stopIndex].getTotalTime()) {
                this.scheduleParallelEvent(event, stopIndex + 1, offset - paths[stopIndex].getTotalTime());
                return;
            }
            this.parallelEvents.get(stopIndex).add(new Pair<>(offset, event));
        } else {
            if (offset < -paths[stopIndex - 1].getTotalTime()) {
                this.scheduleParallelEvent(event, stopIndex - 1, offset + paths[stopIndex - 1].getTotalTime());
                return;
            }
            this.parallelEvents.get(stopIndex - 1).add(new Pair<>(paths[stopIndex - 1].getTotalTime() + offset, event));
        }
    }

    public SequentialCommandGroup generateParallelCommand(int pathIndex) {
        ArrayList<Pair<Double, Command>> currentCommands = this.parallelEvents.get(pathIndex);
        currentCommands.sort(Comparator.comparing(Pair::getFirst));

        LinkedList<Command> spacedCommands = new LinkedList<>();

        for (int i = currentCommands.size() - 1; i > 0; i--) {
            spacedCommands.addFirst(currentCommands.get(i).getSecond());
            spacedCommands.addFirst(new WaitCommand(currentCommands.get(i).getFirst() - currentCommands.get(i - 1).getFirst()));
        }

        if (!spacedCommands.isEmpty()) {
            spacedCommands.addFirst(currentCommands.get(0).getSecond());
            spacedCommands.addFirst(new WaitCommand(currentCommands.get(0).getFirst()));
        }

        return new SequentialCommandGroup(spacedCommands.toArray(new Command[0]));
    }

    private Command getPathCommand(int pathIndex) {
        if (this.finalizedPaths[pathIndex] != null) {
            return this.finalizedPaths[pathIndex];
        }

        Command blockedEvent = this.blockedEvents.get(pathIndex);

        if (pathIndex == this.paths.length) {
            return blockedEvent != null ? blockedEvent : new WaitCommand(0.0);
        }

        Command path = this.paths[pathIndex].alongWith(this.generateParallelCommand(pathIndex));

        if (blockedEvent != null) {
            path = blockedEvent.andThen(path);
        }

        this.finalizedPaths[pathIndex] = path;

        return path;
    }

    @Override
    public final void initialize() {
        currentPath = 0;

        if (this.paths.length == 0) {
            return;
        }

        this.parseEventMarkers();

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
