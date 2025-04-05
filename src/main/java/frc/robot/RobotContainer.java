// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.CoralArm;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.epilogue.Logged;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.ArmRollerSubsystem;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.util.Units;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public final DriveSubsystem driveSubsystem = new DriveSubsystem();

    @Logged(name = "ArmRoller")
    public final ArmRollerSubsystem armRoller = new ArmRollerSubsystem(); // Rename the rollersubsystem class to
                                                                          // armRollerSubsystem

    public final AlgaeArm algaeArm = new AlgaeArm();

    private Mechanism2d mechanisms = new Mechanism2d(5, 3);
    private MechanismRoot2d root = mechanisms.getRoot("root", 2.5, 0.25);

    PositionTracker positionTracker = new PositionTracker();
    private MechanismLigament2d elevatorLigament = root
            .append(new MechanismLigament2d("elevatorStage", Units.inchesToMeters(10), 90,
                    4,
                    new Color8Bit(Color.kOrange)));
    private MechanismLigament2d armLigament = elevatorLigament
            .append(new MechanismLigament2d("armLigament", Units.inchesToMeters(10), 270,
                    5,
                    new Color8Bit(Color.kRed)));
    Elevator elevator = new Elevator(positionTracker, elevatorLigament);
    Arm arm = new Arm(positionTracker, armLigament, elevator::getCarriageComponentPose);
    // CoralSim coralSim = new CoralSim(drivetrain::getPose,
    // arm::getClawComponentPose);
    // The autonomous chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final Joystick driverController =

            new Joystick(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

    // Trigger declarations
    // Trigger button2 = driverController.button(2);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        configureAutoChooser(); // add autonomous options
        SmartDashboard.putData("Autonomous Chooser", autoChooser);

    }

    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        // Set the default command for the drive subsystem to the command provided by
        // factory with the values provided by the joystick axes on the driver
        // controller. The Y axis of the controller is inverted so that pushing the
        // stick away from you (a negative value) drives the robot forwards (a positive
        // value)

        // TURN IS TWISTING, TO TURN BACK CHANGE AXIS 2 TO 0
        // THE INVERTING SYSTEM IS USING THE SPEED NOB WHEN ITS DOWN ITS FACING FORWARD,
        // BUT WHEN ITS UPWARDS IT WILL SWITCH YOUR FRONT SIDE
        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveArcade( // First argument is always the turn (x).
                        driveSubsystem,
                        () -> -driverController.getRawAxis(0) * .9,
                        () -> -driverController.getRawAxis(1) * driverController.getRawAxis(3)));

        elevator.setDefaultCommand(elevator.moveToCurrentGoalCommand());
        // RollerSubsystem. TODO:
        // Add condition that roller may only roll out to eject coral when the arm is in
        // a down position
        // Add another condition that roller roll in or out when the arm is a down
        // position
        // Discuss what button to bind the armRoller to with drivers for rolling in and
        // out

        // driverController.button(2).onTrue(armRoller.runRoller());
        // armRoller.runRollerMotor(Constants.RollerConstants.rollerAlgaeInSpeed);
        // armRoller.runRollerMotor( () ->
        // Constants.RollerConstants.rollerAlgaeInSpeed).withTimeout(1.0);

        new JoystickButton(driverController, OperatorConstants.coralToReef)
                .whileTrue(armRoller.runRollerForward());

        new JoystickButton(driverController, OperatorConstants.intakeGamePiece)
                .whileTrue(armRoller.runRollerReverse());

        new JoystickButton(driverController, OperatorConstants.armUp)
                .whileTrue(algaeArm.ArmUp());

        new JoystickButton(driverController, OperatorConstants.armDown)
                .whileTrue(algaeArm.ArmDown());

        new JoystickButton(driverController, OperatorConstants.elevatorToL3)
                .onTrue(elevator.prepareCoralScoreCommand(ScoreLevel.L3, elevator, arm).withName("MoveElevatorToL3"));

        new JoystickButton(driverController, OperatorConstants.elevatorToL2)
                .onTrue(elevator.prepareCoralScoreCommand(ScoreLevel.L2, elevator, arm).withName("MoveElevatorToL2"));

        new JoystickButton(driverController, OperatorConstants.elevatorToTop)
                .onTrue(elevator.intakeIntoScoreCommand(elevator, arm).withName("MoveElevatorToIntake"));

                new JoystickButton(driverController, 10)
                .onTrue(arm.moveToPositionCommand(() -> CoralArm.ArmPosition.TOP));

        /*
         * new JoystickButton(driverController, OperatorConstants.coralToReefAutomated)
         * .onTrue(armRoller.runForwardAndReverseTimed(0.2, 1));
         */

        new JoystickButton(driverController, OperatorConstants.coralToReefAutomated)
                .onTrue(armRoller.loadCoralCommand(0.2));

        new JoystickButton(driverController, OperatorConstants.scoreL1Coral)
                .onTrue(Commands.sequence(armRoller.runRollerForward().withTimeout(1),
                 algaeArm.ArmDown().withTimeout(1.0)));

        new JoystickButton(driverController, OperatorConstants.scoreL1Coral)
                 .onTrue(arm.resetPositionCommand());
                
        new Trigger(() -> driverController.getPOV() == 0).whileTrue(arm.armUpCommand());

        new Trigger(() -> driverController.getPOV() == 180).whileTrue(arm.armDownCommand());

        new Trigger(() -> driverController.getPOV() == -1).whileTrue(arm.armMotorStop());

        // 2s forward, 1s reverse
        // new JoystickButton(driverController, OperatorConstants.armDownDebouncer)
        // .whileTrue(algaeArm.runDebounceArmDownCmd());

    }

    private void configureAutoChooser() {
        // Set the options to show up in the Dashboard for selecting auto modes. If you
        // add additional auto modes you can add additional lines here with
        // autoChooser.addOption
        autoChooser.setDefaultOption("Do Nothing", Autos.doNothing());
        autoChooser.addOption("Arcade Drive (no rotation @ 50%)", Autos.driveArcadeCmd(driveSubsystem));
        autoChooser.addOption("Drive FWD 2.3 meters", Autos.driveFwdmeters(driveSubsystem, armRoller, algaeArm));
        autoChooser.addOption("Reset Encoders", Autos.resetEncoders(driveSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }
}