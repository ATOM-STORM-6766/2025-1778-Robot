package org.chillout1778;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.Arrays;
import org.chillout1778.commands.AutoRunnerCommand;
import org.chillout1778.commands.TeleopDriveNextCommand;
import org.chillout1778.commands.TeleopSuperstructureCommand;
import org.chillout1778.subsystems.*;

public class Robot extends TimedRobot {

  private static Robot instance = null;

  public static Robot getInstance() {
    if (instance == null) {
      instance = new Robot();
    }
    return instance;
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }

  public boolean isOnRedSide() {
    return SwerveNext.getInstance().getEstimatedPose().getX() > (Constants.Field.FIELD_X_SIZE / 2);
  }

  private final DigitalInput enableCoastModeSwitch =
      new DigitalInput(Constants.DioIds.DISABLE_BREAK_MODE);

  public boolean wasEnabledThenDisabled = false;
  public boolean wasEnabled = false;

  private final Telemetry logger = new Telemetry();

  private final SendableChooser<Trajectory<SwerveSample>> autoChooser = new SendableChooser<>();

  private Robot() {
    // This code tells FMS that we use Kotlin (so that we are part of the
    // end-of-year statistics on which languages people use).
    HAL.report(
        tResourceType.kResourceType_Language,
        tInstances.kLanguage_Kotlin,
        0,
        WPILibVersion.Version);

    // Start logging NetworkTables changes to a USB drive or the RoboRIO.
    DataLogManager.start();

    // Initialize subsystem objects just by referencing them
    Arm.getInstance();
    Elevator.getInstance();
    Intake.getInstance();
    Superstructure.getInstance();
    SwerveNext.getInstance().registerTelemetry(logger::telemeterize);
    ;
    Vision.getInstance();
    Lights.getInstance();

    LogManager.registerSubsystem(Arm.getInstance());
    LogManager.registerSubsystem(Elevator.getInstance());
    LogManager.registerSubsystem(Intake.getInstance());
    LogManager.registerSubsystem(Superstructure.getInstance());
    LogManager.registerSubsystem(Vision.getInstance());

    for (String trajectoryName :
        Arrays.stream(Choreo.availableTrajectories())
            .filter(it -> !it.equals("VariablePoses"))
            .toArray(String[]::new)) {
      autoChooser.addOption(
          trajectoryName, Choreo.<SwerveSample>loadTrajectory(trajectoryName).get());
    }

    autoChooser.onChange(
        t -> {
          autoTrajectory = t;
          initializeAutonomousCommand();
        });

    // Use LogManager to register auto chooser
    LogManager.registerToRobotTab("Auto Chooser", autoChooser);
  }

  public long tickNumber = 0;

  public boolean getWasCoastModeEnabled() {
    return wasCoastModeEnabled;
  }

  public boolean getWasEnabledThenDisabled() {
    return wasEnabledThenDisabled;
  }

  public void setWasEnabledThenDisabled(boolean wasEnabledThenDisabled) {
    this.wasEnabledThenDisabled = wasEnabledThenDisabled;
  }

  @Override
  public void robotPeriodic() {
    tickNumber++;
    CommandScheduler.getInstance().run();
  }

  private boolean wasCoastModeEnabled = false;

  @Override
  public void disabledInit() {
    if (wasEnabled) {
      wasEnabledThenDisabled = true;
    }
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
    autonomousCommand =
        Superstructure.getInstance()
            .makeZeroAllSubsystemsCommand()
            .andThen( // EXTREMELY IMPORTANT TO ZERO
                new AutoRunnerCommand(autoTrajectory))
            .andThen(new TeleopDriveNextCommand(Controls::emptyInputs));
  }

  @Override
  public void autonomousInit() {
    Arm.getInstance().setHasObject(true);
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
    if (!didAutoRun) {
      // The new SwerveNext subsystem handles alliance perspective automatically.
    }

    TeleopDriveNextCommand teleopDriveNextCommand = new TeleopDriveNextCommand(Controls::driverInputs);
    LogManager.registerCommand("Teleop Drive Next", teleopDriveNextCommand);
    
    Superstructure.getInstance().makeZeroAllSubsystemsCommand().schedule();
    SwerveNext.getInstance().setDefaultCommand(teleopDriveNextCommand);
    Superstructure.getInstance().setDefaultCommand(new TeleopSuperstructureCommand());
  }

  @Override
  public void teleopExit() {
    Superstructure.getInstance().removeDefaultCommand();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    Superstructure.getInstance().makeZeroAllSubsystemsCommand().schedule();
  }
}
