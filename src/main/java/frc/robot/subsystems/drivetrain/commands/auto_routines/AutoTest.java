// package frc.robot.subsystems.drivetrain.commands.auto_routines;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import frc.robot.Constants;
// import frc.robot.subsystems.drivetrain.commands.PathResetOdometry;
// import frc.robot.subsystems.drivetrain.commands.TrajectoryFollow;
// import frc.robot.subsystems.intake.IntakeFeeder;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.shooter.commands.LimelightShoot;
// import frc.robot.subsystems.vision.VisionSubsystem;


// public class AutoTest extends SequentialCommandGroup {

//     IntakeFeeder intake_ = IntakeFeeder.getInstance();
//     Shooter shooter_ = Shooter.getInstance();
//     VisionSubsystem vision_ = VisionSubsystem.getInstance();

//     public AutoTest() {
//             addCommands(
//                 new PathResetOdometry("Tarmac-Almost-A"), 
//                 (
//                 new TrajectoryFollow("Tarmac-Almost-A").withTimeout(2)
//                 .raceWith(new RunCommand(intake_::intake, intake_))
//                 // .alongWith(new InstantCommand(() -> shooter_.setVelocityGoal(Constants.kWallShotVelocity, true)))
//                 .alongWith(new LimelightShoot(Constants.kWallShotVelocity, true, true))
//                 )
//                 .andThen(new InstantCommand(intake_::stopIntake))

//         );

//     }
// }
