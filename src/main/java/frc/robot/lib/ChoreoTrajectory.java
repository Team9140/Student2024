package frc.robot.lib;

import com.choreo.lib.ChoreoTrajectoryState;
import java.util.List;

public class ChoreoTrajectory extends com.choreo.lib.ChoreoTrajectory {
    private final List<EventMarker> eventMarkers;

    public ChoreoTrajectory(List<ChoreoTrajectoryState> samples, List<EventMarker> eventMarkers) {
        super(samples);
        this.eventMarkers = eventMarkers;
    }

    public ChoreoTrajectory(List<EventMarker> eventMarkers) {
        this.eventMarkers = eventMarkers;
    }

    @Override
    public ChoreoTrajectory flipped() {
        return (ChoreoTrajectory) super.flipped();
    }

    public List<EventMarker> getEventMarkers() {
        return eventMarkers;
    }
}
