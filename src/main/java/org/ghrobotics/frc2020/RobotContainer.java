package org.ghrobotics.frc2020;

import java.util.Arrays;

import org.ghrobotics.frc2020.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class RobotContainer {
  private Drivetrain drive = new Drivetrain();

  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
        2, 3);
    config.setKinematics(drive.getKinematics());

    // HE DIDNT ADD A VOLTAGE CONSTRAINT WHICH IS SUGGESTED IN DOCUMENTATION
    config.addConstraint(new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(1.16, 3.37, 0.573),
        drive.getKinematics(), 10));

    // His Trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d()),
            new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))),
        config
    );

    // Drive Straight Trajectory
    Trajectory trajDriveStraight = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(), new Pose2d(2, 0, new Rotation2d(0))),
        config);

    // Drive Forward and left Trajectory
    Trajectory trajForwardRight = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(new Pose2d(), new Pose2d(2, 0.75, new Rotation2d(0))),
      config);

    // Turn 90 degrees
    Trajectory trajQuarterTurn = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(new Pose2d(), new Pose2d(1.25, 0.5, Rotation2d.fromDegrees(90))),
      config);


    RamseteCommand command = new RamseteCommand(
        trajectory,
        drive::getPose,
        new RamseteController(2, .7),
        drive.getFeedforward(),
        drive.getKinematics(),
        drive::getSpeeds,
        drive.getLeftPIDController(),
        drive.getRightPIDController(),
        drive::setOutputVolts,
        drive
    );

    return command.andThen(() -> drive.setOutputVolts(0, 0));
  }

  public void reset() {
    drive.reset();
  }
}
