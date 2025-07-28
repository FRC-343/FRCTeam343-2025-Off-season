package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.Auto.Test;
import frc.robot.bobot_state2.BobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DrivePerpendicularToPoseCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.field.FieldUtils;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Leds.LED;
import frc.robot.subsystems.Leds.LEDInput;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision2.Vision;
// import frc.robot.subsystems.vision.VisionIO;
// import frc.robot.subsystems.vision.VisionIOPhotonVision;
// import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.CommandCustomController;
import frc.robot.util.Constant;
import frc.robot.util.Constant.elevatorConstants;
import frc.robot.util.PoseUtils;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // @SuppressWarnings("unused")
  // private final Vision vision;

  private final Elevator elevator;

  private final Climber climber;

  private final Intake intake;

  private final LEDInput led;

  private final LED m_LEDs = LED.getInstance();

  // Controller
  private final CommandCustomController controller = new CommandCustomController(0);

  private final CommandCustomController controller2 = new CommandCustomController(1);

  private final DriverAutomationFactory m_Automation;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    new BobotState();
    new Vision();
    led = new LEDInput();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        m_Automation = new DriverAutomationFactory(controller, controller2, drive);
        elevator = new Elevator();
        intake = new Intake();
        climber = new Climber();

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera3Name, VisionConstants.robotToCamera3,
        // drive::getPose));

        m_Automation = new DriverAutomationFactory(controller, controller2, drive);

        elevator = new Elevator();
        intake = new Intake();
        climber = new Climber();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIO() {},
        //         new VisionIO() {},
        //         new VisionIO() {},
        //         new VisionIO() {});
        m_Automation = new DriverAutomationFactory(controller, controller2, drive);

        elevator = new Elevator();
        intake = new Intake();
        climber = new Climber();
        break;
    }
    configureNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // autoChooser.addOption("Test Auto Pathing", new Test(elevator, intake));

    // SmartDashboard.putData(MetalUtils.getQuickReefOne());

    // Configure the button bindings
    configureOperatorButton();
    configureDefaults();
    configureDriverButtons();

    // configureAutomation();

    //     SmartDashboard.putString("QuickReefOne", MetalUtils.getQuickReefOneTAGv());
    //     SmartDashboard.putString("QuickReefTwo", MetalUtils.getQuickReefTwoTAGv());
    //     SmartDashboard.putString("QuickReefThree", MetalUtils.getQuickReefThreeTAGv());
  }

  private void configureNamedCommands() {

    NamedCommands.registerCommand("L4Elevator", elevator.setElevatorPositionL4());

    NamedCommands.registerCommand("L3ELevator", elevator.setElevatorPositionL3());
    NamedCommands.registerCommand("L2Elevator", elevator.setElevatorPositionL2());
    NamedCommands.registerCommand("Feed", elevator.setElevatorPositionFeed());
    NamedCommands.registerCommand("Outtake", intake.runForTime(-.8, .5));

    NamedCommands.registerCommand("Launch", intake.runForTime(.8, .8));

    NamedCommands.registerCommand("HPIntake", intake.HPintake());

    // DEBUG COMMANDS
    NamedCommands.registerCommand(
        "DEBUG INTAKE STOP PT1",
        new InstantCommand(() -> elevator.overrideBeambreakObstructedCommand(true)));
    NamedCommands.registerCommand(
        "DEBUG INTAKE STOP PT2",
        new InstantCommand(() -> elevator.overrideBeambreakObstructedCommand(false)));

    // TEST COMMANDS

    NamedCommands.registerCommand("Quick Stop", intake.BeamWait());

    NamedCommands.registerCommand(
        "Drive test",
        new DriveToPoseCommand(
            drive,
            () ->
                PoseUtils.plusRotation(
                    FieldUtils.getClosestReef().rightPole.getPerpendicularOffsetPose(.5),
                    Rotation2d.kPi)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  //   private void configureAutomation() {
  //     // BobotState.nearHumanPlayer().whileTrue(intake.HPintake()).onFalse(intake.stopCommand());
  //   }
  private void configureDefaults() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
  }

  private void configureDriverButtons() {
    controller.povLeft().onTrue(new InstantCommand(() -> BobotState.updateNearness("HPS")));
    controller.povRight().onTrue(new InstantCommand(() -> BobotState.updateNearness("Reef")));
    controller.povDown().onTrue(new InstantCommand(() -> BobotState.updateNearness("")));
    controller
        .povUp()
        .whileTrue(
            new DriveToPoseCommand(
                drive,
                () ->
                    PoseUtils.plusRotation(
                        FieldUtils.getBargeTag().leftCage.getPose(), Rotation2d.kZero)));

    controller
        .rightTrigger()
        .whileTrue(
            DrivePerpendicularToPoseCommand.withJoystickRumble(
                drive,
                () -> FieldUtils.getClosestReef().rightPole.getPose(),
                () -> -controller.getLeftYSquared(),
                Commands.parallel(
                    controller.rumbleOnOff(1, 0.25, 0.25, 2),
                    controller2.rumbleOnOff(1, 0.25, 0.25, 2))));

    controller
        .a()
        .whileTrue(
            new DriveToPoseCommand(
                drive,
                () ->
                    PoseUtils.plusRotation(
                        FieldUtils.getClosestHPSTag().hps.getPerpendicularOffsetPose(0.3),
                        Rotation2d.kZero)));

    controller
        .b()
        .whileTrue(
            DrivePerpendicularToPoseCommand.withJoystickRumble(
                drive,
                () ->
                    PoseUtils.plusRotation(
                        FieldUtils.getClosestHPSTag().hps.getPose(), Rotation2d.kPi),
                () -> -controller.getLeftYSquared(),
                Commands.parallel(
                    controller.rumbleOnOff(1, 0.25, 0.2, 2),
                    controller2.rumbleOnOff(1, 0.25, 0.2, 2))));

    controller
        .b()
        .and(controller.rightBumper())
        .whileTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller
        .leftTrigger()
        .whileTrue(
            new DriveToPoseCommand(
                    drive,
                    () ->
                        PoseUtils.plusRotation(
                            FieldUtils.getClosestReef().rightPole.getPerpendicularOffsetPose(.45),
                            Rotation2d.kPi))
                .andThen(controller2.rumbleOnOff(2, .25, .2, 2)));

    controller
        .rightBumper()
        .whileTrue(
            DrivePerpendicularToPoseCommand.withJoystickRumble(
                drive,
                () ->
                    PoseUtils.plusRotation(
                        FieldUtils.getBargeTag().leftCage.getPose(), Rotation2d.kPi),
                () -> -controller.getLeftYSquared(),
                Commands.parallel(
                    controller.rumbleOnOff(1, 0.25, 0.2, 2),
                    controller2.rumbleOnOff(1, 0.25, 0.2, 2))));
  }

  private void configureOperatorButton() {

    // Test Controlls

    controller2
        .leftTrigger()
        .and(BobotState.elevatorAtFeed())
        .debounce(.5)
        .onTrue(intake.HPintake());

    controller2
        .leftTrigger()
        .and(BobotState.elevatorAtFeed().negate())
        .whileTrue(elevator.setElevatorPosition(elevatorConstants.FEED));
    // "Intake" Controlls

    controller2
        .povLeft()
        .whileTrue(new RunCommand(() -> m_LEDs.Left()))
        .onFalse(new InstantCommand(() -> m_LEDs.cycleRedWhitePattern()));

    controller2
        .povRight()
        .whileTrue(new RunCommand(() -> m_LEDs.Right()))
        .onFalse(new InstantCommand(() -> m_LEDs.cycleRedWhitePattern()));

    controller2
        .leftBumper()
        .and(controller2.rightBumper().negate())
        .whileTrue(intake.setPercentOutputThenStopCommand(.8));

    controller2
        .rightTrigger()
        .and(controller2.leftBumper().negate())
        .and(controller2.rightBumper().negate())
        .whileTrue(intake.setPercentOutputThenStopCommandT1(-.8));

    // Elevator buttons

    controller2
        .y()
        .and(controller2.rightBumper().negate())
        .whileTrue(elevator.setElevatorPosition(elevatorConstants.Barge));

    controller2
        .rightStick()
        .and(controller2.rightBumper().negate())
        .whileTrue(elevator.setElevatorPosition(Constant.elevatorConstants.L2AlgeaLevel));

    controller2
        .a()
        .and(controller2.rightBumper().negate())
        .whileTrue(elevator.setElevatorPosition(Constant.elevatorConstants.L3Level));

    controller2
        .x()
        .and(controller2.rightBumper().negate())
        .whileTrue(elevator.setElevatorPosition(Constant.elevatorConstants.L4Level));

    controller2
        .b()
        .and(controller2.rightBumper().negate())
        .whileTrue(elevator.setElevatorPosition(Constant.elevatorConstants.L2Level));

    // Climber Buttons

    controller2
        .rightBumper()
        .whileTrue(new RunCommand(() -> m_LEDs.servoState()))
        .onFalse(new InstantCommand(() -> m_LEDs.Idle()));

    controller2
        .pov(180)
        .and(controller2.rightBumper())
        .whileTrue(climber.runBack(-.2))
        .whileFalse(climber.runBack(0));

    controller2
        .pov(0)
        .and(controller2.rightBumper())
        .whileTrue(climber.setPercentOutputCommand(1))
        .whileFalse(climber.setPercentOutputCommand(0));

    controller2.y().and(controller2.rightBumper()).onTrue(climber.goForRotForward(64));
    controller2.a().and(controller2.rightBumper()).onTrue(climber.goForRotBack(28));

    controller2.b().and(controller2.rightBumper()).onTrue(climber.Disengage());
    controller2.x().and(controller2.rightBumper()).onTrue(climber.Engage());

    // Auto Intake test

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void playMusic() {
    intake.playMusic();
    drive.playMusic();
  }

  public void pauseMusic() {
    intake.pauseMusic();
    drive.pauseMusic();
  }

  public void Automation() {
    BobotState.intakeBeam().onTrue(intake.HPintake());
  }
}
