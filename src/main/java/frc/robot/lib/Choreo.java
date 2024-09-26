package frc.robot.lib;

import com.google.gson.Gson;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

public class Choreo extends com.choreo.lib.Choreo {
    private static final Gson gson = new Gson();

    public static ChoreoTrajectory getTrajectory(String trajName) {
        var traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
        var traj_file = new File(traj_dir, trajName + ".traj");

        return loadFile(traj_file);
    }

    private static ChoreoTrajectory loadFile(File path) {
        try {
            var reader = new BufferedReader(new FileReader(path));

            return gson.fromJson(reader, ChoreoTrajectory.class);
        } catch (Exception ex) {
            DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
        }
        return null;
    }
}
