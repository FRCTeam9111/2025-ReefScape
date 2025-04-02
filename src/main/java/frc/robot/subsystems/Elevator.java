package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseLinearMechanism;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.PositionTracker;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CoralArm;
import frc.robot.Constants.CoralArm.*;
import frc.robot.GlobalStates;
import frc.robot.Constants.ElevatorConstants.*;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.epilogue.Epilogue;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import frc.robot.ScoreLevel;
import frc.robot.subsystems.AlgaeArm.ArmState;
import frc.robot.CoralSim;
import frc.robot.CoralSim.CoralSimLocation;


@Logged
public class Elevator extends SubsystemBase implements BaseLinearMechanism<ElevatorPosition> {
    @Log
    private final SparkMax motor;

    private SparkMaxConfig motorConfig;

    public static ScoreLevel lastScore = ScoreLevel.None;

    @Log(groups = "control")
    private final ProfiledPIDController pidController = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.MOVEMENT_CONSTRAINTS);

    @Log(groups = "control")
    private final ElevatorFeedforward feedforwardController = new ElevatorFeedforward(ElevatorConstants.kS,
    ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    @Logged(name="Elevator: Lift IOInfo")
    private final ElevatorIOInfo ioInfo = new ElevatorIOInfo();
    @Logged
    public static class ElevatorIOInfo {
        public double liftAtPositionInMeters = 0.0;
        public double liftDesiredPositionInMeters = ElevatorPosition.BOTTOM.value;
        public double liftSimVelocityInMetersPerSec = 0.0;
        public double liftVelocityInMetersPerSec = 0.0;
        public double liftAppliedVolts = 0.0;
        public double liftCurrentAmps = 0.0;
    }


      private void updateElevatorIOInfo() {
        ioInfo.liftAtPositionInMeters = motor.getEncoder().getPosition();
        ioInfo.liftVelocityInMetersPerSec = motor.get();     // note: does not get updated during simulation use corresponding liftSimVelocity
        ioInfo.liftAppliedVolts = motor.getAppliedOutput();
        ioInfo.liftCurrentAmps = motor.getOutputCurrent();
        // note: ioInfo.liftDesiredPositionMeters updated with the operator control commands

        if (Robot.isSimulation()) {
            ioInfo.liftSimVelocityInMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        }
    }

    /**
     * The representation of the "elevator" for simulation. (even though this is a
     * rotational mechanism w.r.t. its setpoints, we still control it as a linear
     * mechanism since that is the cloest physical mechanism to this)
     */
    private final ElevatorSim elevatorSim = new ElevatorSim(
        ElevatorConstants.MOTOR_GEARBOX_REPR,
        ElevatorConstants.GEARING,
        ElevatorConstants.MASS_KG,
        ElevatorConstants.DRUM_RADIUS_METERS,
        ElevatorConstants.MIN_HEIGHT_METERS,
        ElevatorConstants.MAX_HEIGHT_METERS,
            true,
            ElevatorPosition.BOTTOM.value);

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;

    private double simVelocity = 0.0;

    private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
    private final MutDistance sysidPositionMeasure = Meters.mutable(0);
    private final MutLinearVelocity sysidVelocityMeasure = MetersPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine;

    private final PositionTracker positionTracker;
    private final MechanismLigament2d ligament;

    @Log
    private boolean initialized;

    public Elevator(PositionTracker positionTracker, MechanismLigament2d ligament) {
        motorConfig = new SparkMaxConfig();

        motorConfig
                .inverted(ElevatorConstants.MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
        motorConfig.encoder
                .positionConversionFactor(ElevatorConstants.ENCODER_ROTATIONS_TO_METERS)
                .velocityConversionFactor(ElevatorConstants.ENCODER_ROTATIONS_TO_METERS / 60.0);

        motor = new SparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(1).per(Seconds), Volts.of(5), null, null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("primary")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(motor.getAppliedOutput(), Volts))
                                    .linearPosition(sysidPositionMeasure.mut_replace(getPosition(), Meters))
                                    .linearVelocity(sysidVelocityMeasure.mut_replace(getVelocity(), MetersPerSecond));
                        },
                        this));

        this.positionTracker = positionTracker;
        this.ligament = ligament;

        positionTracker.setElevatorPositionSupplier(this::getPosition);
        setDefaultCommand(moveToCurrentGoalCommand());
    }

    @Override
    public void periodic() {
        // note: default command moveToSetPointCommand() automatically runs
        updateElevatorIOInfo();
        SmartDashboard.putData(this);
        System.out.println("elevator encoder" + getPosition() );
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(motor.getAppliedOutput());
        elevatorSim.update(0.020);
        motor.getEncoder().setPosition(elevatorSim.getPositionMeters());
        simVelocity = elevatorSim.getVelocityMetersPerSecond();

        ligament.setLength(getPosition());
    }

    public boolean getInitialized() {
        return initialized;
    }

    @Log(groups = "components")
    public Pose3d getFrameComponentPose() {
        return new Pose3d(0.14, 0, 0.13, new Rotation3d());
    }

    @Log(groups = "components")
    public Pose3d getStageComponentPose() {
        Transform3d transform = new Transform3d();
        if (getPosition() > 0.706) {
            transform = new Transform3d(0, 0, getPosition() - 0.706, new Rotation3d());
        }
        return new Pose3d(0.14, 0, 0.169, new Rotation3d()).plus(transform);
    }

    @Log(groups = "components")
    public Pose3d getCarriageComponentPose() {
        return new Pose3d(0.14, 0, 0.247 + getPosition(), new Rotation3d());
    }

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public double getVelocity() {
        if (RobotBase.isReal())
            return motor.getEncoder().getVelocity();
        else
            return simVelocity;
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(ElevatorPosition.BOTTOM.value);
        initialized = true;
    }

    @Override
    public void setVoltage(double voltage) {
        System.out.println("in setVoltage");
        voltage = MathUtil.clamp(voltage, -12, 12);
        voltage = Utils.applySoftStops(voltage, getPosition(), ElevatorConstants.MIN_HEIGHT_METERS, ElevatorConstants.MAX_HEIGHT_METERS);

        if (voltage < 0
                && positionTracker.getElevatorPosition() < Constants.ElevatorConstants.MOTION_LIMIT
                && positionTracker.getArmAngle() < 0) {
            voltage = 0;
            System.out.println("voltage is 0");
        }

        if (!GlobalStates.INITIALIZED.enabled()) {
            voltage = 0.0;
            System.out.println("initialized not enabled");
        }

        motor.setVoltage(voltage);

    }

    @Override
    public Command moveToCurrentGoalCommand() {
        System.out.println("moveToCurrentGoalCommand");
        SmartDashboard.putNumber("Elevator/Feedback Voltage", feedbackVoltage);
SmartDashboard.putNumber("Elevator/Feedforward Voltage", feedforwardVoltage);
SmartDashboard.putNumber("Elevator/Setpoint Position", pidController.getSetpoint().position);
SmartDashboard.putNumber("Elevator/Setpoint Velocity", pidController.getSetpoint().velocity);
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforwardController.calculate(pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("elevator.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier) {
        System.out.println("moveToPositionCommand");
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand()
                        .until(() -> pidController.atGoal()))
                .withTimeout(3)
                .withName("elevator.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(this::atGoal)).withName("elevator.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
                .withName("elevator.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
                .withName("elevator.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("elevator.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("elevator.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(motor::stopMotor)
                .andThen(() -> {
                    motorConfig.idleMode(IdleMode.kCoast);
                    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                })
                .finallyDo((d) -> {
                    motorConfig.idleMode(IdleMode.kBrake);
                    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("elevator.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("elevator.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("elevator.sysIdDynamic");
    }

    public Command resetControllersCommand() {
        return Commands.runOnce(() -> pidController.reset(getPosition()))
                .andThen(Commands.runOnce(() -> pidController.setGoal(getPosition())));
    }

    public boolean atGoal() {
        return pidController.atGoal();
    }

    public void moveElevatorUp()
    {
        motor.set(ArmConstants.ARM_SPEED_UP);
       
    }

    public void armHoldUp()
    {
        motor.set(ArmConstants.ARM_HOLD_UP);

    }
    public void moveElevatorDown()
    {
        motor.set(ArmConstants.ARM_SPEED_DOWN);

    }

    public void armHoldDown()
    {
        motor.set(ArmConstants.ARM_HOLD_DOWN);
    }


    public Command moveElevatorUpoCommand()
    {
        return this.startEnd(this::moveElevatorUp, this::armHoldUp);
    }


    public Command moveElevatorDownCommand()
    {
        return this.startEnd(this::moveElevatorDown, this::armHoldDown);
    }


    public Command prepareCoralScoreCommand(ScoreLevel level, Elevator elevator, Arm arm) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        System.out.println("Preparing to score at level: " + level);
        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
            }
            case TOP -> {
                elevatorPosition = ElevatorPosition.TOP;
                armPosition = ArmPosition.L3;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ScoreLevel");
            }
        }

        return Commands.runOnce(() -> {
            lastScore = level;
        }).andThen(Commands.parallel(
                        arm.moveToPositionCommand(() -> armPosition).asProxy(),
                        Commands.waitSeconds(0.5)
                                .andThen(elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())));
    }

    public static Command intakeIntoScoreCommand(Elevator elevator, Arm arm) {
        return Commands.sequence(
            // Move elevator to TOP position
            elevator.moveToPositionCommand(() -> ElevatorConstants.ElevatorPosition.TOP),
            // Move arm to BOTTOM position
            arm.moveToPositionCommand(() -> CoralArm.ArmPosition.BOTTOM),
            // Final elevator position adjustment
            elevator.moveToPositionCommand(() -> ElevatorConstants.ElevatorPosition.INTAKE),

            elevator.moveToPositionCommand(() -> ElevatorConstants.ElevatorPosition.TOP)
        ).withName("intakeIntoScore");
    }

}