package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units;

public class Drivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private Field2d field = new Field2d();
    // ADIS16470_IMU green_gyro;


    private final SwerveRequest.SysIdSwerveTranslation SysIDTranslate = new SwerveRequest.SysIdSwerveTranslation();

    private SysIdRoutine translateRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Units.Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SysIDTranslate.withVolts(volts)),
                    null,
                    this));

    private final SwerveRequest.SysIdSwerveRotation SysIDRotate = new SwerveRequest.SysIdSwerveRotation();

    private SysIdRoutine rotateRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Units.Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SysIDRotate.withVolts(volts)),
                    null,
                    this));

    private final SwerveRequest.SysIdSwerveSteerGains SysIDSteer = new SwerveRequest.SysIdSwerveSteerGains();

    private SysIdRoutine steerRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Units.Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SysIDSteer.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine routineToApply = translateRoutine;

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routineToApply.dynamic(direction);
    }

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
                                   SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // green_gyro = new ADIS16470_IMU();
        // this.m_pigeon2.getConfigurator().apply(new
        // GyroTrimConfigs().withGyroScalarZ(OdometryUpdateFrequency))

    }

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // green_gyro = new ADIS16470_IMU();
    }

    public void setField(Field2d field) {
        this.field = field;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pigeon yaw", this.m_pigeon2.getYaw().getValueAsDouble());
        // SmartDashboard.putNumber("green yaw", this.green_gyro.getAngle());
        // SmartDashboard.putNumber("diff", this.m_pigeon2.getYaw().getValueAsDouble() - this.green_gyro.getAngle());
        SmartDashboard.putNumber("module angle reference", this.Modules[0].getSteerMotor().getClosedLoopReference().getValueAsDouble() % (Math.PI * 2.0));
        SmartDashboard.putNumber("module angle", this.Modules[0].getSteerMotor().getPosition().getValueAsDouble() % (Math.PI * 2.0));
        SmartDashboard.putNumber("module angle error", this.Modules[0].getSteerMotor().getClosedLoopError().getValueAsDouble());

        SmartDashboard.putNumber("wheel velocity reference", this.Modules[0].getDriveMotor().getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("wheel velocity", this.Modules[0].getDriveMotor().getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("wheel velocity error", this.Modules[0].getDriveMotor().getClosedLoopError().getValueAsDouble());

        this.field.setRobotPose(this.getState().Pose);

        SmartDashboard.putNumber("steer error", this.getState().Pose.getRotation().getRadians());

        SmartDashboard.putString("pose2d", this.getState().Pose.toString());
        SmartDashboard.putNumber("wheel current", this.Modules[0].getDriveMotor().getStatorCurrent().getValueAsDouble());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command resetGyros() {
        return this.runOnce(() -> {
            // green_gyro.setGyroAngleZ(0.0);
            m_pigeon2.setYaw(0.0);
        });
    }
}