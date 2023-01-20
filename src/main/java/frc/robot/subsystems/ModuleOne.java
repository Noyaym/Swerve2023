package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;

/**
 * Subsystem consisting of one module.
 */
public class ModuleOne extends SubsystemBase {

    private SwerveModule module;

    /**
     * Constructs one module as a subsystem
     */
    public ModuleOne() {

        module = new SwerveModule(Constants.Offsets.BACK_RIGHT_OFFSEST, Constants.ModuleConst.BACK_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_CANCODER_ID, Constants.ModuleConst.BACK_RIGHT_SET_INVERT_TYPE);

        SmartDashboard.putData("set vel",
                new RunCommand(() -> module.setVel(SmartDashboard.getNumber("target velocity", 0)), this)
                        .andThen(new InstantCommand(() -> module.setVel(0), this)));

        SmartDashboard.putData("set angle",
                new RunCommand(() -> module.setAngle(SmartDashboard.getNumber("target angle", 0)), this)
                .andThen(new InstantCommand(() -> module.setPowerSteerMotor(0), this)));

        SmartDashboard.putData("Calibrate", new InstantCommand(() ->
        module.calibrate()));
    }

    /**
     * Gets wheel velocity.
     * @return wheel velocity
     */
    public double getVelocity() {
        return module.getVelocity();
    }

    /**
     * Gets wheel angle (degrees).
     * @return wheel angle in degrees
     */
    public double getAngle() {
        return module.getAngle();
    }

    /**
     * Gets sensor position (pulses) of steer motor.
     * @return sensor position (pulses) of steer motor
     */
    public double getSteerSelectedSensorPosition() {
        return module.getSteerSelectedSensorPosition();
    }

    /**
     * Gets wheel offset.
     * @return wheel offset
     */
    public double getOffset() {
        return module.getOffset();
    }

    /**
     * Gets the closed loop error of steer motor.
     * @return closed loop error of steer motor
     */
    public double getError() {
        return module.getSteerMotor().getClosedLoopError();
    }

    /**
     * Checks if steer error is negative.
     * @return is steer motor error negative
     */
    public boolean isErrorNegative() {
        return module.getSteerMotor().getClosedLoopError()<0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putNumber("target velocity", 0);
        SmartDashboard.putNumber("target angle", 0);
        builder.addDoubleProperty("velocity", this::getVelocity, null);
        builder.addDoubleProperty("angle", this::getAngle, null);

        builder.addDoubleProperty("offset", this::getOffset, null);
        builder.addDoubleProperty("pulses", this::getSteerSelectedSensorPosition, null);
        builder.addDoubleProperty("error", this::getError, null);
        builder.addBooleanProperty("is negative", this::isErrorNegative, null);

    }
}
