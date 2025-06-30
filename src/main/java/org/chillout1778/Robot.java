package org.chillout1778;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.chillout1778.commands.AutoRunnerCommand;
import org.chillout1778.commands.TeleopDriveCommand;
import org.chillout1778.commands.TeleopSuperstructureCommand;
import org.chillout1778.subsystems.*;

public class Robot extends TimedRobot {
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }    public static boolean isOnRedSide() {        return Swerve.getInstance().getEstimatedPose().getX() > (Constants.Field.FIELD_X_SIZE / 2);
    }

    public static boolean wasEnabledThenDisabled = false;
    public static boolean wasEnabled = false;
    
    // Add a simple enabled state tracker  
    private static boolean currentlyEnabled = false;

    public static boolean isCurrentlyEnabled() {
        return currentlyEnabled;
    }

    private final DigitalInput enableCoastModeSwitch = new DigitalInput(Constants.DioIds.DISABLE_BREAK_MODE);

    private final SendableChooser<Trajectory<SwerveSample>> autoChooser = new SendableChooser<>();

    public Robot() {
        // This code tells FMS that we use Java (converted from Kotlin)
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Java, 0, WPILibVersion.Version);

        // Start logging NetworkTables changes to a USB drive or the RoboRIO.
        DataLogManager.start();

        // Initialize subsystem objects
        Arm.getInstance();
        Elevator.getInstance();
        Intake.getInstance();
        Superstructure.getInstance();
        Swerve.getInstance();
        Vision.getInstance();
        Lights.getInstance();

        Shuffleboard.getTab("Subsystems").add(Arm.getInstance());
        Shuffleboard.getTab("Subsystems").add(Elevator.getInstance());
        Shuffleboard.getTab("Subsystems").add(Intake.getInstance());        Shuffleboard.getTab("Subsystems").add(Superstructure.getInstance());
        Shuffleboard.getTab("Subsystems").add(Vision.getInstance());
        
        for (String trajectoryName : Choreo.availableTrajectories()) {
            if (!trajectoryName.equals("VariablePoses")) {
                autoChooser.addOption(trajectoryName, Choreo.<SwerveSample>loadTrajectory(trajectoryName).get());
            }
        }

        autoChooser.onChange(t -> {
            autoTrajectory = t;
            initializeAutonomousCommand();
        });

        Shuffleboard.getTab("Robot").add(autoChooser);
    }

    public static long tickNumber = 0;

    @Override
    public void robotPeriodic() {
        tickNumber++;
        CommandScheduler.getInstance().run();
    }

    private boolean wasCoastModeEnabled = false;

    @Override
    public void disabledInit() {
        if (wasEnabled) wasEnabledThenDisabled = true;
        wasEnabled = false;
    }

    @Override
    public void disabledPeriodic() {
        boolean pressed = !enableCoastModeSwitch.get();
        if (!wasCoastModeEnabled && pressed) { // rising edge
            Elevator.getInstance().setCoastEnabled(true);
            Arm.getInstance().setCoastEnabled(true);
            wasCoastModeEnabled = true;
        } else if (wasCoastModeEnabled && !pressed) { // falling edge
            Elevator.getInstance().setCoastEnabled(false);
            Arm.getInstance().setCoastEnabled(false);
            wasCoastModeEnabled = false;
        }
    }

    @Override
    public void disabledExit() {
        Elevator.getInstance().setCoastEnabled(false);
        Arm.getInstance().setCoastEnabled(false);
        wasEnabled = true;
        wasEnabledThenDisabled = false;
    }

    // This is code for running autonomous Commands only in auto mode
    private boolean didAutoRun = false;
    private Trajectory<SwerveSample> autoTrajectory;

    private Command autonomousCommand = new InstantCommand();

    private void initializeAutonomousCommand() {
        autonomousCommand = Superstructure.getInstance().makeZeroAllSubsystemsCommand().andThen( // EXTREMELY IMPORTANT TO ZERO
            new AutoRunnerCommand(autoTrajectory)
        ).andThen(new TeleopDriveCommand(() -> Controls.emptyInputs));
    }

    @Override
    public void autonomousInit() {
        Arm.getInstance().hasObject = true;
        Arm.getInstance().autoTimer.reset();
        Arm.getInstance().autoTimer.start();
        didAutoRun = true;
        autonomousCommand.schedule();
    }

    @Override
    public void autonomousExit() {
        autonomousCommand.cancel();
    }

    @Override
    public void teleopInit() {
        if (!didAutoRun)
            Swerve.getInstance().setGyroAngle(isRedAlliance() ? Math.PI : 0.0);
        Superstructure.getInstance().makeZeroAllSubsystemsCommand().schedule();
        Swerve.getInstance().setDefaultCommand(new TeleopDriveCommand(Controls::driverInputs));
        Superstructure.getInstance().setDefaultCommand(new TeleopSuperstructureCommand());
    }

    @Override
    public void teleopExit() {
        Swerve.getInstance().removeDefaultCommand();
        Superstructure.getInstance().removeDefaultCommand();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        Superstructure.getInstance().makeZeroAllSubsystemsCommand().schedule();
        Swerve.getInstance().setGyroAngle(0.0);
    }
}
