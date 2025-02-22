
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmRollerSubsystem.RollerState;;

public class AlgaeArm extends SubsystemBase {
    private final SparkMax algaeMotor;

    // ENUM Arm State and declaration
    public static enum ArmState
    {
        UP,
        DOWN
    };

    @Logged(name = "Arm State")
    ArmState armState;

    public AlgaeArm(){
        algaeMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);

        algaeMotor.setCANTimeout(250);

        SparkMaxConfig algaeConfig = new SparkMaxConfig();
        algaeConfig.voltageCompensation(10);
        algaeConfig.smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
        algaeConfig.idleMode(IdleMode.kBrake);
        algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void runMotorUp(){
        algaeMotor.set(ArmConstants.ARM_SPEED_UP);
    }

    public void runMotorDown(){
        algaeMotor.set(ArmConstants.ARM_SPEED_DOWN);
    }

    private void KeepArmUp() {
        armState = ArmState.UP;
        algaeMotor.set(ArmConstants.ARM_HOLD_UP);
      }


      private void KeepArmDown() {
        armState = ArmState.DOWN;
        algaeMotor.set(ArmConstants.ARM_HOLD_DOWN);
      }
    
    // Commands -----

    public Command runArmUp() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return this.startRun(this::runMotorUp, this::KeepArmUp)
                .withName("Roller/CMD/runRollerForward");
    }

    public Command runArmDown() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return this.startRun(this::runMotorDown, this::KeepArmDown)
                .withName("Roller/CMD/runRollerForward");
    }
}
