package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConst;

/**
 * utility class, represents one swerve module
 */
public class SwerveModule {

    private final TalonFX moveMotor;
    private final TalonFX steerMotor;
    private final CANCoder encoder;
    private final SimpleMotorFeedforward ff;
    private double offset;

    /**
     * Constructs new swerve module.
     * @param offset wheel offset
     * @param moveMotorID id of move motor
     * @param steerMotorID id of steer motor
     * @param CANCoderID id of CANConder
     * @param setInverted is steer motor inverted
     */
    public SwerveModule(double offset, int moveMotorID, int steerMotorID, int CANCoderID, boolean setInverted) {
        this.moveMotor = new WPI_TalonFX(moveMotorID);
        this.steerMotor = new WPI_TalonFX(steerMotorID);
        this.encoder = new WPI_CANCoder(CANCoderID);
        this.offset = offset;

        moveMotor.configFactoryDefault();
        steerMotor.configFactoryDefault();
        steerMotor.setInverted(setInverted);
        moveMotor.config_kP(0, ModuleConst.mVel_Kp);
        moveMotor.config_kI(0, ModuleConst.mVel_Ki);
        moveMotor.config_kD(0, ModuleConst.mVel_Kd);

        steerMotor.config_kP(0, ModuleConst.mAngle_Kp);
        steerMotor.config_kI(0, ModuleConst.mAngle_Ki);
        steerMotor.config_kD(0, ModuleConst.mAngle_Kd);

        this.ff = new SimpleMotorFeedforward(ModuleConst.MOVE_MOTOR_Ks, ModuleConst.MOVE_MOTOR_Kv);
    }

    //steer motor related

    /**
     * Gets steer motor.
     * @return steer motor
     */
    public TalonFX getSteerMotor() {
        return steerMotor;
    }

    /**
     * Sets neutral mode of steer motor.
     * @param isBrake is the desired neutral mode brake
     */
    public void setNeutraleModeSteerMotor(boolean isBrake) {
        steerMotor.setNeutralMode(isBrake? NeutralMode.Brake:NeutralMode.Coast);
    }

    /**
     * Gets wheel angle (degrees).
     * @return wheel angle in degrees
     */
    public double getAngle() {
        double value  = encoder.getAbsolutePosition() - offset;
        if (value<0) value = 360 + value;
        return value;
    }

    /**
     * Gets wheel angle (normalized to 360).
     * @return normalized wheel angle in degrees
     */
    public double getAngleNotWithin360() {
        return encoder.getAbsolutePosition() - offset;
    }

    /**
     * Gets wheel angle (Rotation2ds).
     * @return wheel angle in Rotation2d
     */
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition() - offset);
    }

    /**
     * Gets sensor position (pulses) of steer motor.
     * @return sensor position (pulses) of steer motor
     */
    public double getSteerSelectedSensorPosition() {
        return steerMotor.getSelectedSensorPosition();
    }

    /**
     * Converts angle from degrees to motor pulses.
     * @param angle the desired wheel angle
     * @return steer motor position in pulses
     */
    public double convertAngle2Pulse(double angle) {
        return angle * ModuleConst.PULSE_PER_ANGLE;
    }

    /**
     * Converts offset from degrees to motor pulses.
     * @param offset the wheel offset
     * @return offset in pulses
     */
    public double convertOffset2Pulse() {
        return offset * ModuleConst.PULSE_PER_ANGLE;
    }

    /**
     * Calculates feed forward of steer motor.
     * @param difference the difference between desired wheel angle to current one
     * @return feed forward value
     */
    public double feedForwardSetAngle(double difference) {
        if (Math.abs(difference)<Constants.ModuleConst.TOLERANCE_STEER) {
            return 0;
        }
        return Math.signum(difference)*ModuleConst.mAngle_Ks;
    }

    /**
     * Sets power of steer motor.
     * @param power desired power for steer motor
     */
    public void setPowerSteerMotor(double power) {
        steerMotor.set(ControlMode.PercentOutput, power);
    }

    /**
     * Sets wheel angle.
     * @param angle desired angle in degrees
     */
    public void setAngle(double angle) {
        double dif = angle - getAngle();
        double difference = Utils.optimizeAngleDemacia(dif);
        steerMotor.set(ControlMode.Position,
                steerMotor.getSelectedSensorPosition()+convertAngle2Pulse(difference),
                DemandType.ArbitraryFeedForward, 
                feedForwardSetAngle(difference));
        SmartDashboard.putNumber("difference", difference);
        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber("selctedSensorPosition", 
        steerMotor.getSelectedSensorPosition()/ModuleConst.PULSE_PER_ANGLE);
    }

    /**
     * Gets wheel offset.
     * @return wheel offset
     */
    public double getOffset() {
        return offset;
    }

    //move motor related

    /**
     * Gets move motor.
     * @return move motor
     */
    public TalonFX getMoveMotor() {
        return moveMotor;
    }

     /**
     * Sets neutral mode of move motor.
     * @param isBrake is the desired neutral mode brake
     */
    public void setNeutraleModeMoveMotor(boolean isBrake) {
        moveMotor.setNeutralMode(isBrake? NeutralMode.Brake:NeutralMode.Coast);
    }

    /**
     * Gets wheel velocity.
     * @return wheel velocity
     */
    public double getVelocity() {
        return moveMotor.getSelectedSensorVelocity() / ModuleConst.PULSE_PER_METER * 10;
    }

    /**
     * Sets wheel velocity.
     * @param velocity desired wheel velocity
     */
    public void setVel(double velocity) {
        moveMotor.set(ControlMode.Velocity, velocity * ModuleConst.PULSE_PER_METER / 10,
                DemandType.ArbitraryFeedForward, ff.calculate(velocity));
    }

    /**
     * Sets move motor power.
     * @param power desired move motor power
     */
    public void setPowerVelocity(double power) {
        moveMotor.set(ControlMode.PercentOutput, power);
    }

    //additional

    /**
     * Gets distance driven by wheel.
     * @return distance driven by wheel
     */
    public double getDistance() {
        return moveMotor.getSelectedSensorPosition() / ModuleConst.PULSE_PER_METER;
    }

    /**
     * Gets module state.
     * @return module state
     */
    public SwerveModuleState getState() { // gets the state of a module
        double velocity = getVelocity();
        Rotation2d angle = Rotation2d.fromDegrees(getAngle());

        return new SwerveModuleState(velocity, angle);
    }

    /**
     * Gets module position.
     * @return module position
     */
    public SwerveModulePosition getPosition() { // gets the position of a module.
        double distanceDriven = getDistance();
        return new SwerveModulePosition(distanceDriven, getAngleRotation2d());
    }

    /**
     * Calibrates module - sets zero point.
     */
    public void calibrate() {
        offset = encoder.getAbsolutePosition();
        steerMotor.setSelectedSensorPosition(0);
    }

}
