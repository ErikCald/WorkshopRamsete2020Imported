package org.ghrobotics.frc2020.subsystems;

// import com.kauailabs.navx.frc.AHRS;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;



public class Drivetrain extends SubsystemBase {

  private static final double kGearRatio = 7.29;
  private static final double kWheelRadiusInches = 3.0;

  // CANSparkMax leftMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  // CANSparkMax rightMaster = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

  // CANSparkMax leftSlave = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  // CANSparkMax rightSlave = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

  WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
  WPI_TalonSRX rightMaster = new WPI_TalonSRX(2);

  WPI_TalonSRX leftSlave = new WPI_TalonSRX(3);
  WPI_TalonSRX rightSlave = new WPI_TalonSRX(4);


  // AHRS gyro = new AHRS(SPI.Port.kMXP);

  PigeonIMU pigeon;
  PigeonIMU.FusionStatus fusionStatus;

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.583); // Cad Measured value: 0.69
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1.16, 3.37, 0.573);   // His values 0.3, 1.96, 0.06

  PIDController leftPIDController = new PIDController(2.15, 0, 0);  // His value 2.95
  PIDController rightPIDController = new PIDController(2.15, 0, 0);  

  Pose2d pose = new Pose2d();

  public Drivetrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    pigeon = new PigeonIMU(leftSlave);
    pigeon.setFusedHeading(0d, 100);
    fusionStatus = new PigeonIMU.FusionStatus();

    pigeon.setYaw(0, 10);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 1000);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 1000);
  }

  public Rotation2d getHeading() {

    pigeon.getFusedHeading(fusionStatus);
    return Rotation2d.fromDegrees(fusionStatus.heading);
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        talonVelocityToMetersPerSecond(leftMaster.getSelectedSensorVelocity()),
        talonVelocityToMetersPerSecond(rightMaster.getSelectedSensorVelocity())
    );
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    leftMaster.set(ControlMode.PercentOutput, leftVolts / 12);
    rightMaster.set(ControlMode.PercentOutput, rightVolts / 12);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getLeftEncoderMeters(), getRightEncoderMeters());
  }

  private double getLeftEncoderMeters() {
    double rawValue = leftMaster.getSelectedSensorPosition();
    double filteredValue = rawValue * 1.0341d;
    return talonPosistionToMeters(filteredValue);
  }

  private double getRightEncoderMeters() {
    double rawValue = rightMaster.getSelectedSensorPosition();
    double filteredValue = rawValue * 1.0341d;
    return talonPosistionToMeters(filteredValue);
  }



  private double talonPosistionToMeters(double talonPosisiton) {
    double circumference = Math.PI * 0.1524;
    double metersPerTick = circumference / 4096;
    return talonPosisiton * metersPerTick;
  }

  private double talonVelocityToMetersPerSecond(double talonVelocity) {
    return talonPosistionToMeters(talonVelocity * 10);
  }

  private double metersToTalonPosisiton(double meters) {
    double circumference = Math.PI * 0.1524;
    double ticksPerMeter = 4096 / circumference;
    return meters * ticksPerMeter;
  }

  private double metersPerSecondToTalonVelocity(double metersPerSecond) {
    return metersToTalonPosisiton(metersPerSecond * 0.1);
  }
}
