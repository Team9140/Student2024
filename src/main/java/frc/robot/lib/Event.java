package frc.robot.lib;

public class Event {
    private final String type;
    private final EventAttributes data;

    public Event(String type, EventAttributes data) {
        this.type = type;
        this.data = data;
    }

    public String getType() {
        return type;
    }

    public EventAttributes getData() {
        return data;
    }
}
