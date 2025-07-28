package frc.robot;

// import frc.robot.commands.PositionWithCoralStation;
// import frc.robot.commands.PositionWithReef;
import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.vision.OffsetTags;
import frc.robot.util.CommandCustomController;

public class DriverAutomationFactory {
  private final CommandCustomController driverController;
  private final CommandCustomController operatorController;

  private final Drive drive;

  public DriverAutomationFactory(
      CommandCustomController driverController,
      CommandCustomController operatorController,
      Drive drive) {
    this.driverController = driverController;
    this.operatorController = operatorController;
    this.drive = drive;
  }

  // public Command quickCoralPath() {
  //   return MetalUtils.getCoralTag().getDeferredCommand();
  // }

  // public Command quickCoralAssist() {
  //   return Commands.defer(
  //       () ->
  //           new PositionWithCoralStation(
  //               () -> -driverController.getLeftX(), drive, MetalUtils.getCoralTag()),
  //       Set.of(drive));
  // }

  // public Command CoralPath() {
  //   return MetalUtils.getOtherCoralTag().getDeferredCommand();
  // }

  // public Command CoralAssist() {
  //   return Commands.defer(
  //       () ->
  //           new PositionWithCoralStation(
  //               () -> -driverController.getLeftX(), drive, MetalUtils.getOtherCoralTag()),
  //       Set.of(drive));
  // }

  // public Command quickReefOnePath() {
  //   return MetalUtils.getQuickReefOne().getDeferredCommand();
  // }

  // public Command quickReefOneAssist() {
  //   return Commands.defer(
  //       () ->
  //           new PositionWithReef(
  //               () -> -driverController.getLeftX(), drive, MetalUtils.getQuickReefOne()),
  //       Set.of(drive));
  // }

  // public Command quickReefTwoPath() {
  //   return MetalUtils.getQuickReefTwo().getDeferredCommand();
  // }

  // public Command quickReefTwoAssist() {
  //   return Commands.defer(
  //       () ->
  //           new PositionWithReef(
  //               () -> -driverController.getLeftX(), drive, MetalUtils.getQuickReefTwo()),
  //       Set.of(drive));
  // }

  // public Command quickReefThreePath() {
  //   return MetalUtils.getQuickReefThree().getDeferredCommand();
  // }

  // public Command quickReefThreeAssist() {
  //   return Commands.defer(
  //       () ->
  //           new PositionWithReef(
  //               () -> -driverController.getLeftX(), drive, MetalUtils.getQuickReefThree()),
  //       Set.of(drive));
  // }

  // public Command processor() {
  //   return OffsetTags.PROCESSOR.getDeferredCommand();
  // }

  // public Command processorAssist() {
  //   return Commands.defer(
  //       () -> new PositionWithReef(() -> -driverController.getLeftX(), drive,
  // OffsetTags.PROCESSOR),
  //       Set.of(drive));
  // }

  // public Command ReefOnePath() {
  //   return MetalUtils.getReefOne().getDeferredCommand();
  // }

  // public Command ReefOneAssist() {
  //   return Commands.defer(
  //       () ->
  //           new PositionWithReef(
  //               () -> -driverController.getLeftX(), drive, MetalUtils.getReefOne()),
  //       Set.of(drive));
  // }

  // public Command ReefTwoPath() {
  //   return MetalUtils.getReefTwo().getDeferredCommand();
  // }

  // public Command ReefTwoAssist() {
  //   return Commands.defer(
  //       () ->
  //           new PositionWithReef(
  //               () -> -driverController.getLeftX(), drive, MetalUtils.getReefTwo()),
  //       Set.of(drive));
  // }

  // public Command ReefThreePath() {
  //   return MetalUtils.getReefThree().getDeferredCommand();
  // }

  // public Command ReefThreeAssist() {
  //   return Commands.defer(
  //       () ->
  //           new PositionWithReef(
  //               () -> -driverController.getLeftX(), drive, MetalUtils.getReefThree()),
  //       Set.of(drive));
  // }
}
