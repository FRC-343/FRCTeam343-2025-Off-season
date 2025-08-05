package frc.robot.bobot_state2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bobot_state2.varc.BargeTagTracker;
import frc.robot.bobot_state2.varc.HPSTagTracker;
import frc.robot.bobot_state2.varc.ProcessorTagTracker;
import frc.robot.bobot_state2.varc.ReefTagTracker;
import frc.robot.bobot_state2.varc.TargetAngleTracker;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.vision.PoseObservation;
import frc.robot.util.VirtualSubsystem;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import org.littletonrobotics.junction.Logger;

/**
 * Class full of static variables and methods that store robot state we'd need across mulitple
 * subsystems. It's called {@link #BobotState} as to not conflict with WPILib's {@link
 * edu.wpi.first.wpilibj.RobotState}
 */
public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  private static final Queue<PoseObservation> poseObservations = new LinkedBlockingQueue<>(20);

  private static boolean grabMode =
      false; // False means Coral, True means Algae. Better ways of implementing this is just for
  // offseason comp. This is going to be used for automation.

  private static Pose2d globalPose = new Pose2d(); // Robots position on the field.

  private static boolean
      elevatorBeam; // Elevator beam break, not needed on offseason bot, but was used to determine
  // if the elevator was clear since we passed coral through it.

  private static boolean
      climberState; // Climber limit switch, not needed on offseason bot(yet), but was used to
  // determine if the climber was on the limit switch or not.

  private static boolean
      atWantedPerpPose; // Robots perpendicular position in relation to whatever Apriltag we are
  // lining up to.

  private static boolean
      atWantedRot; // Robots rotation in relation to whatever Apriltag we are lining up to

  private static boolean
      atWantedParaPose; // Robots parallel position in relation to whatever Apriltag we are lining
  // up to

  private static boolean
      elevatorHigherThanL3; // Used to see if the elevator was higher than our L3 level so we could
  // slow the robot down(shouldnt be an issue on offseason bot yet to see)

  private static boolean
      elevatorHigherThanL4; // Used to see if the elevator was higher than our L4 level so we could
  // slow the robot down(shouldnt be an issue on offseason bot yet to see

  private static boolean
      intakeBeam1; // Intake beam break, not needed on offseason bot, but was used to see if there
  // was a coral in the intake to run the intake wheels(should be replaced by a
  // limit switch for simplicity on offseason bot)

  private static boolean
      intakeBeam2; // Intake beam break, not needed on offseason bot, but was used to see if there
  // was a coral in the intake to run the intake wheels(should be replaced by a
  // limit switch for simplicity on offseason bot)

  private static double elevatorPose; // Elevators current position.

  private static double armPose; // Arms current position.

  //keeps track of the closest tag on the reef to the robot
  private static ReefTagTracker reefTracker = new ReefTagTracker();
  //keeps track of the tag on the human processing station
  private static HPSTagTracker hpsTracker = new HPSTagTracker();
  //keeps track of closest tag on barge
  private static BargeTagTracker bargeTracker = new BargeTagTracker();
  //keeps track of the tag for the processor
  private static ProcessorTagTracker processorTracker = new ProcessorTagTracker();

  //test item
  /*private static String wantNearWhat = "";*/

  //tracks the target angle for alignment
  private static List<TargetAngleTracker> autoAlignmentTrackers =
    //list of the trackers
      List.of(
          BobotState.hpsTracker,
          BobotState.reefTracker,
          BobotState.processorTracker,
          BobotState.bargeTracker);

  //boolean for the grab mode
  public static void updateGrabMode(boolean mode) {
    BobotState.grabMode = mode;
  }

  //test case for updating the nearest target
  /* public static void updateNearness(String test) {
    BobotState.wantNearWhat = test;
  }*/

  //tracks the elevator pose
  public static void updateElevatorPose(double pose) {
    BobotState.elevatorPose = pose;
  }

  //tracks the arm pose
  public static void updateArmPose(double pose) {
    BobotState.armPose = pose;
  }

  //COMMENTING EXCERISE WITH THE KIDS
  public static void updateIntakeBeam1(boolean beam) {
    BobotState.intakeBeam1 = beam;
  }

  //COMMENTING EXCERISE WITH THE KIDS
  public static void updateIntakeBeam2(boolean beam) {
    BobotState.intakeBeam2 = beam;
  }

  //updating elevator state, if elevator is past L3
  public static void updateElevatorState(boolean state) {
    BobotState.elevatorHigherThanL3 = state;
  }

  //updating elevator state: electric boogaloo, same but L4
  public static void updateElevatorState2(boolean state) {
    BobotState.elevatorHigherThanL4 = state;
  }

  //updating if elevator is clear
  public static void updateElevatorBeam(boolean beam) {
    BobotState.elevatorBeam = beam;
  }

  //updating the perpendicular wanted pose
  public static void updateWantedPose(boolean perpPoseWanted) {
    BobotState.atWantedPerpPose = perpPoseWanted;
  }

  //updating the parallel wanted pose
  public static void updateWantedParaPose(boolean paraPoseWanted) {
    BobotState.atWantedParaPose = paraPoseWanted;
  }

  //updating the wanted rotation
  public static void updateWantedRot(boolean rotWanted) {
    BobotState.atWantedRot = rotWanted;
  }

  //self-explanatory
  public static void updateClimberState(boolean state) {
    BobotState.climberState = state;
  }

  //LOOK AT LATER
  public static void offerVisionObservation(PoseObservation observation) {
    BobotState.poseObservations.offer(observation);
  }

  //SAME
  public static Queue<PoseObservation> getVisionObservations() {
    return BobotState.poseObservations;
  }

  //sel-explanatory
  public static void updateGlobalPose(Pose2d pose) {
    BobotState.globalPose = pose;
  }

  //self-explanatory
  public static Pose2d getGlobalPose() {
    return BobotState.globalPose;
  }

  //test for getting what the robot is near
  public static String getNearWhat() {
    return BobotState.wantNearWhat;
  }

  //s-e
  public static boolean getClimberState() {
    return BobotState.climberState;
  }

  //s-e
  public static Trigger atWantedPose() {
    return new Trigger(
        () -> BobotState.atWantedParaPose && BobotState.atWantedPerpPose && BobotState.atWantedRot);
  }

  //checks what team's side you are on
  public static Trigger onTeamSide() {
    return new Trigger(
        () ->
            FieldUtils.getAlliance() == Alliance.Blue
                ? getGlobalPose().getX() < FieldConstants.fieldLength / 2.0
                : getGlobalPose().getX() > FieldConstants.fieldLength / 2.0);
  }

  //essentially boolean, outputs whichever is true
  public static Trigger intakeBeam() {
    return new Trigger(() -> (BobotState.intakeBeam1 || BobotState.intakeBeam2));
  }

  //checks if the arm pose is past a certain point within rotation (past or before 90deg)
  public static Trigger armPoseNoGo() {
    return new Trigger(() -> BobotState.armPose > 2);
  }

  //sending rotation to the processor
  public static Rotation2d getRotationToProcessor() {
    return BobotState.processorTracker.getRotationTarget();
  }

  //gets your rotation to the reef
  public static Rotation2d getRotationToClosestReef() {
    return BobotState.reefTracker.getRotationTarget();
  }

  //gets rotation towards the HPS
  public static Rotation2d getRotationToClosestHPS() {
    return BobotState.hpsTracker.getRotationTarget();
  }

  //gets rotation to barge
  public static Rotation2d getRotationToClosestBarge() {
    return BobotState.bargeTracker.getRotationTarget();
  }

  //gets distance from the HPS
  public static double getDistanceMetersFromClosestHPS() {
    return BobotState.hpsTracker.getDistanceMeters();
  }

  //test case to trigger lights (can do something w/ later)
  public static Trigger humanPlayerShouldThrow() {
    return new Trigger(() -> BobotState.hpsTracker.getDistanceMeters() < 0.5);
  }

  //boolean if near the HPS
  public static Trigger nearHumanPlayer() {
    return new Trigger(() -> BobotState.hpsTracker.getDistanceMeters() < 1);
  }

  //getting the nearest tag to the robot
  public static TargetAngleTracker getClosestAlignmentTracker() {
    return autoAlignmentTrackers.stream()
        .reduce((a, b) -> a.getDistanceMeters() < b.getDistanceMeters() ? a : b)
        .get();
  }

  //returns the state of the elevator beam
  public static Trigger ElevatorBeam() {
    //(testing) System.out.println(BobotState.elevatorBeam);
    return new Trigger(() -> BobotState.elevatorBeam);
  }

  //triggers slowing down the elevator if past L3
  public static Trigger ElevatorSlowdown1() {
    return new Trigger(() -> BobotState.elevatorHigherThanL3);
  }

  //triggers slowing down the elevator if past L4
  public static Trigger ElevatorSlowdown2() {
    return new Trigger(() -> BobotState.elevatorHigherThanL4);
  }

  //getter for elevator's position during travel
  public static double ElevatorPose() {
    return BobotState.elevatorPose;
  }

  //boolean for is the elevator is at the feeding position
  public static Trigger elevatorAtFeed() {
    return new Trigger(() -> (BobotState.elevatorPose < .5));
  }

  //getter for the arm pose
  public static double armPose() {
    return BobotState.armPose;
  }

  //logs
  @Override
  public void periodic() {

    Logger.recordOutput(logRoot + "Wanted to be near", wantNearWhat);

    Logger.recordOutput(logRoot + "Arm Pose", armPose);

    Logger.recordOutput(logRoot + "Intake Beam 1", intakeBeam1);

    Logger.recordOutput(logRoot + "Intake Beam 2", intakeBeam2);

    Logger.recordOutput(
        logRoot + "Elevator Slowdown", elevatorHigherThanL3 || elevatorHigherThanL4);

    Logger.recordOutput(logRoot + "Climber limit", climberState);

    Logger.recordOutput(logRoot + "Elevator Beam", elevatorBeam);

    Logger.recordOutput(logRoot + "Wanted Perp Pose", atWantedPerpPose);

    Logger.recordOutput(logRoot + "Wanted Para Pose", atWantedParaPose);

    Logger.recordOutput(logRoot + "Wanted Rot", atWantedRot);

    Logger.recordOutput(logRoot + "Wanted Pose", atWantedPose());

    Logger.recordOutput(logRoot + "Elevator pose", elevatorPose);

    Logger.recordOutput(logRoot + "Elevator at feed", elevatorAtFeed());
    {
      processorTracker.update();

      String calLogRoot = logRoot + "Processor";
      Logger.recordOutput(calLogRoot + "Processor", FieldUtils.getProcessorFace().tag);
      Logger.recordOutput(
          calLogRoot + "TargetAngleDeg", processorTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calLogRoot + "TargetAngleRad", processorTracker.getRotationTarget().getRadians());
    }

    {
      reefTracker.update();

      String calcLogRoot = logRoot + "Reef/";
      Logger.recordOutput(calcLogRoot + "ClosestTag", FieldUtils.getClosestReef().tag);
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", reefTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", reefTracker.getRotationTarget().getRadians());
      Logger.recordOutput(calcLogRoot + "Left Pole", FieldUtils.getClosestReef().leftPole);
      Logger.recordOutput(calcLogRoot + "Right Pole", FieldUtils.getClosestReef().rightPole);
    }

    {
      hpsTracker.update();

      String calcLogRoot = logRoot + "HPS/";
      Logger.recordOutput(calcLogRoot + "Closest Tag", FieldUtils.getClosestHPSTag().hps);
      Logger.recordOutput(calcLogRoot + "Distance", BobotState.hpsTracker.getDistanceMeters());
      Logger.recordOutput(calcLogRoot + "IsClose", BobotState.nearHumanPlayer());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", hpsTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", hpsTracker.getRotationTarget().getRadians());
    }

    {
      bargeTracker.update();

      String calcLogRoot = logRoot + "Barge/";
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", bargeTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", bargeTracker.getRotationTarget().getRadians());
    }

    {
      String calcLogRoot = logRoot + "ClosestAlignment/";
      Logger.recordOutput(
          calcLogRoot + "Type", getClosestAlignmentTracker().getClass().getSimpleName());
    }
  }

  //used for vision within sim
  @Override
  public void simulationPeriodic() {}
}
