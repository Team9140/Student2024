// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final Drivetrain drivetrain = TunerConstants.DriveTrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drivetrain.MAX_TELEOP_SPEED * 0.1)
            .withRotationalDeadband(Constants.Drivetrain.MAX_TELEOP_ANGULAR_RATE * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    Field2d m_field = new Field2d();

    ChoreoTrajectory traj;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        traj = Choreo.getTrajectory("TestPath");

        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                       .withVelocityX(-driverController.getLeftY() * Constants.Drivetrain.MAX_TELEOP_SPEED)
                       .withVelocityY(-driverController.getLeftX() * Constants.Drivetrain.MAX_TELEOP_SPEED)
                       .withRotationalRate(-driverController.getRightX() * Constants.Drivetrain.MAX_TELEOP_ANGULAR_RATE)).ignoringDisable(true));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        boolean isRed = DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Red).equals(DriverStation.Alliance.Red);
        PIDController thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        drivetrain.seedFieldRelative(isRed ? traj.getFlippedInitialPose() : traj.getInitialPose());

        m_field.getObject("traj").setPoses(
                isRed ? traj.getFlippedInitialPose() : traj.getInitialPose(),
                isRed ? traj.getFlippedFinalPose() : traj.getFinalPose()
        );
        m_field.getObject("trajPoses").setPoses(
                isRed ? traj.flipped().getPoses() : traj.getPoses()
        );

        SmartDashboard.putData(m_field);

        SmartDashboard.putString("starting position", (isRed ? traj.getFlippedInitialPose() : traj.getInitialPose()).toString());
        SmartDashboard.putString("desired ending position", (isRed ? traj.getFlippedFinalPose() : traj.getFinalPose()).toString());

        Command swerveCommand = Choreo.choreoSwerveCommand(
                traj,
                () -> drivetrain.getState().Pose,
                new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0),
                new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0),
                thetaController,
                (ChassisSpeeds speeds) -> drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
                () -> isRed, // Whether to mirror the path based on alliance (this assumes the path is created for the blue alliance)
                drivetrain // The subsystem(s) to require
        );

        return Commands.sequence(
                Commands.runOnce(() -> drivetrain.seedFieldRelative(isRed ? traj.getFlippedInitialPose() : traj.getInitialPose())),
                swerveCommand,
                drivetrain.run(() -> drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0))))
        );
    }
}
