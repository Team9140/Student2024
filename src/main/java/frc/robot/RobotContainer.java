// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.RunAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;


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

    private final Field2d m_field = new Field2d();

    private final Candle candle;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        this.candle = Candle.getInstance();
        SmartDashboard.putData("Field", this.m_field);

        this.drivetrain.setField(this.m_field);

        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link CommandPS4Controller
     * PS4} controllers or {@link CommandJoystick Flight
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
    public Command getAutonomousCommand(String autoName)
    {
        RunAuto auto = null;
        switch (autoName){
            case "TestPath":
                auto = new RunAuto("TestPath", this.drivetrain, 1, true, this.m_field);
                auto.setBlockedEvent(new PrintCommand("First shot."), 0);
                auto.scheduleParallelEvent(new PrintCommand("First intake."), 0, 2.19);
                auto.scheduleParallelEvent(new PrintCommand("Second shot."), 0, 3.98);
                auto.scheduleParallelEvent(new PrintCommand("Second intake."), 0, 5.33);
                auto.scheduleParallelEvent(new PrintCommand("Third shot."), 1, -3.60);
                auto.scheduleParallelEvent(new PrintCommand("Third intake."), 1, -2.00);
                auto.setBlockedEvent(new PrintCommand("Final shot."), 1);
                auto.setBlockedEvent(this.candle.setColor(0,20,30, 1), 0);
                auto.scheduleParallelEvent(this.candle.setColor(0, 255, 0, 1), 1, -2.00);
                auto.scheduleParallelEvent(this.candle.setColor(0, 255, 0, 1), 0, 2.19);
                auto.scheduleParallelEvent(this.candle.setColor(0, 255, 0, 1), 0, 5.33);
                auto.scheduleParallelEvent(this.candle.setColor(0, 255, 0, 1), 0, 3.98);
                auto.scheduleParallelEvent(this.candle.setColor(0, 255, 0, 1), 1, -3.60);
                auto.setBlockedEvent(this.candle.setColor(0, 255, 0, 1), 1);
                break;
            case "SourceSide":
                auto = new RunAuto("SourceSide", this.drivetrain, 1, true, this.m_field);
                break;
            case "AmpSide":
                auto = new RunAuto("AmpSide", this.drivetrain, 1, true, m_field);
                break;
            default:
                this.m_field.getObject("traj").setPoses();
                this.m_field.getObject("trajPoses").setPoses();
        }
        return auto;
    }
}
