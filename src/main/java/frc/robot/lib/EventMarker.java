package frc.robot.lib;

public class EventMarker {
    private final double timestamp;
    private final Event command;

    public EventMarker(double timestamp, Event command) {
        this.timestamp = timestamp;
        this.command = command;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public Event getCommand() {
        return command;
    }
}
