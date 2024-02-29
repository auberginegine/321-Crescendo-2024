/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.drivetrain;

import org.robolancers321.Constants.SwerveModuleConstants;
import org.robolancers321.util.VirtualSubsystem;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends VirtualSubsystem {
  /*
   * Singletons
   */

  private static SwerveModule frontLeft = null;

  public static SwerveModule getFrontLeft() {
    if (frontLeft == null)
      frontLeft = new SwerveModule("front left", 2, 1, 9, false, true, false, -0.447021);

    return frontLeft;
  }

  private static SwerveModule frontRight = null;

  public static SwerveModule getFrontRight() {
    if (frontRight == null)
      frontRight = new SwerveModule("front right", 4, 3, 10, true, true, false, -0.362793);

    return frontRight;
  }

  private static SwerveModule backRight = null;

  public static SwerveModule getBackRight() {
    if (backRight == null)
      backRight = new SwerveModule("back right", 6, 5, 11, false, true, false, 0.363281);

    return backRight;
  }

  private static SwerveModule backLeft = null;

  public static SwerveModule getBackLeft() {
    if (backLeft == null)
      backLeft = new SwerveModule("back left", 8, 7, 12, false, true, false, -0.090088);

    return backLeft;
  }

  /*
   * Implementation
   */

  private final String id;

  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final CANcoder turnEncoder;

  private final SparkPIDController driveController;
  private final PIDController turnController;

  private SwerveModuleState commandedState;

  private SwerveModule(
      String id,
      int driveMotorPort,
      int turnMotorPort,
      int turnEncoderPort,
      boolean invertDriveMotor,
      boolean invertTurnMotor,
      boolean invertTurnEncoder,
      double turnEncoderOffset) {
    this.id = id;

    this.driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
    this.driveEncoder = this.driveMotor.getEncoder();
    this.driveController = this.driveMotor.getPIDController();
    this.configDrive(driveMotorPort, invertDriveMotor);

    this.turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);
    this.turnEncoder = new CANcoder(turnEncoderPort);
    this.turnController = new PIDController(SwerveModuleConstants.kTurnP, SwerveModuleConstants.kTurnI, SwerveModuleConstants.kTurnD);
    this.configTurn(
        turnMotorPort, turnEncoderPort, invertTurnMotor, invertTurnEncoder, turnEncoderOffset);

    this.commandedState = new SwerveModuleState();
  }

  private void configDrive(int driveMotorPort, boolean invertDriveMotor) {
    this.driveMotor.setInverted(invertDriveMotor);
    this.driveMotor.setIdleMode(IdleMode.kBrake);
    this.driveMotor.setSmartCurrentLimit(40);
    this.driveMotor.enableVoltageCompensation(12);

    this.driveEncoder.setPosition(0.0);
    this.driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDrivePositionConversionFactor);
    this.driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveVelocityConversionFactor);

    this.driveController.setP(SwerveModuleConstants.kDriveP);
    this.driveController.setI(SwerveModuleConstants.kDriveI);
    this.driveController.setD(SwerveModuleConstants.kDriveD);
    this.driveController.setFF(SwerveModuleConstants.kDriveFF);

    this.driveMotor.burnFlash();
  }

  private void configTurn(
      int turnMotorPort,
      int turnEncoderPort,
      boolean invertTurnMotor,
      boolean invertTurnEncoder,
      double turnEncoderOffset) {
    this.turnMotor.setInverted(invertTurnMotor);
    this.turnMotor.setIdleMode(IdleMode.kBrake);
    this.turnMotor.setSmartCurrentLimit(40);
    this.turnMotor.enableVoltageCompensation(12);

    CANcoderConfiguration config = SwerveModuleConstants.kCANCoderConfig;
    config.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
    config.MagnetSensor.withMagnetOffset(turnEncoderOffset);
    config.MagnetSensor.withSensorDirection(
        invertTurnEncoder
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive);
    this.turnEncoder.getConfigurator().apply(config);

    this.turnController.enableContinuousInput(-Math.PI, Math.PI);

    this.turnMotor.burnFlash();
  }

  public double getDriveVelocityMPS() {
    return this.driveEncoder.getVelocity();
  }

  public double getTurnAngleRotations() {
    return this.turnEncoder.getPosition().getValueAsDouble();
  }

  public double getTurnAngleRad() {
    return 2 * Math.PI * this.getTurnAngleRotations();
  }

  public double getTurnAngleDeg() {
    return 360.0 * this.getTurnAngleRotations();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        this.driveEncoder.getPosition(), Rotation2d.fromRadians(this.getTurnAngleRad()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        this.getDriveVelocityMPS(), Rotation2d.fromRadians(this.getTurnAngleRad()));
  }

  protected void dangerouslyRunDrive(double speed) {
    this.driveMotor.set(speed);
  }

  protected void dangerouslyRunTurn(double speed) {
    this.turnMotor.set(speed);
  }

  protected void update(SwerveModuleState desiredState) {
    this.commandedState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(this.getTurnAngleRad()));
  }

  protected void doSendables() {
    SmartDashboard.putNumber(this.id + " position m", this.getPosition().distanceMeters);

    SmartDashboard.putNumber(this.id + " drive vel (m/s)", this.getDriveVelocityMPS());
    SmartDashboard.putNumber(this.id + " turn angle (deg)", this.getTurnAngleDeg());
  }

  protected static void initTuning() {
    SmartDashboard.putNumber(
        "module drive kp", SmartDashboard.getNumber("module drive kp", SwerveModuleConstants.kDriveP));
    SmartDashboard.putNumber(
        "module drive ki", SmartDashboard.getNumber("module drive ki", SwerveModuleConstants.kDriveI));
    SmartDashboard.putNumber(
        "module drive kd", SmartDashboard.getNumber("module drive kd", SwerveModuleConstants.kDriveD));
    SmartDashboard.putNumber(
        "module drive kff", SmartDashboard.getNumber("module drive kff", SwerveModuleConstants.kDriveFF));

    SmartDashboard.putNumber("module turn kp", SmartDashboard.getNumber("module turn kp", SwerveModuleConstants.kTurnP));
    SmartDashboard.putNumber("module turn ki", SmartDashboard.getNumber("module turn ki", SwerveModuleConstants.kTurnI));
    SmartDashboard.putNumber("module turn kd", SmartDashboard.getNumber("module turn kd", SwerveModuleConstants.kTurnD));
  }

  protected void tune() {
    double tunedDriveP = SmartDashboard.getNumber("module drive kp", SwerveModuleConstants.kDriveP);
    double tunedDriveI = SmartDashboard.getNumber("module drive ki", SwerveModuleConstants.kDriveI);
    double tunedDriveD = SmartDashboard.getNumber("module drive kd", SwerveModuleConstants.kDriveD);
    double tunedDriveFF = SmartDashboard.getNumber("module drive kff", SwerveModuleConstants.kDriveFF);

    this.driveController.setP(tunedDriveP);
    this.driveController.setI(tunedDriveI);
    this.driveController.setD(tunedDriveD);
    this.driveController.setFF(tunedDriveFF);

    double tunedTurnP = SmartDashboard.getNumber("module turn kp", SwerveModuleConstants.kTurnP);
    double tunedTurnI = SmartDashboard.getNumber("module turn ki", SwerveModuleConstants.kTurnI);
    double tunedTurnD = SmartDashboard.getNumber("module turn kd", SwerveModuleConstants.kTurnD);

    this.turnController.setPID(tunedTurnP, tunedTurnI, tunedTurnD);
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber(this.id + " ref angle", this.commandedState.angle.getDegrees());

    this.driveController.setReference(this.commandedState.speedMetersPerSecond, ControlType.kVelocity);

    this.turnController.setSetpoint(this.commandedState.angle.getRadians());

    double turnOutput =
        MathUtil.clamp(this.turnController.calculate(this.getTurnAngleRad()), -1.0, 1.0);

    this.turnMotor.set(turnOutput);
  }
}
