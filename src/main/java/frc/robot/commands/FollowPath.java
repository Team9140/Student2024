package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

public class FollowPath extends Command {
    private ChoreoTrajectory trajectory;
    private ChoreoControlFunction controller;
    private Timer m_timer;
    private Drivetrain m_drive;
    private SwerveRequest.ApplyChassisSpeeds m_request;
    private boolean mirror;

    public FollowPath(String name, Drivetrain drive, boolean mirror) {
        m_drive = drive;
        m_request = new SwerveRequest.ApplyChassisSpeeds();
        addRequirements(drive);
        this.trajectory = Choreo.getTrajectory(name);
        this.mirror = mirror;
        this.controller = Choreo.choreoSwerveController(
                new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0),
                new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0),
                new PIDController(Constants.AutoConstants.kPThetaController, 0.0, 0.0)
        );
        this.m_timer = new Timer();
    }

    public double getTotalTime() {
        return this.trajectory.getTotalTime();
    }

    @Override
    public void initialize() {
        this.m_timer.restart();

        if (this.mirror && DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red)) {
            this.trajectory = this.trajectory.flipped();
        }
    }

    @Override
    public void execute() {
        ChoreoTrajectoryState state = this.trajectory.sample(this.m_timer.get());
        this.m_drive.setControl(m_request.withSpeeds(this.controller.apply(this.m_drive.getState().Pose, state)));
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            this.m_drive.setControl(m_request.withSpeeds(new ChassisSpeeds()));
        } else {
            this.m_drive.setControl(m_request.withSpeeds(trajectory.getFinalState().getChassisSpeeds()));
        }
    }

    @Override
    public boolean isFinished() {
        return this.m_timer.hasElapsed(this.trajectory.getTotalTime());
    }

    public Pose2d getInitialPose() {
        return this.trajectory.getInitialPose();
    }

    public List<Pose2d> getPoses() {
        return List.of(this.trajectory.getPoses());
    }

    public Pose2d getFinalPose() {
        return this.trajectory.getFinalPose();
    }
}
