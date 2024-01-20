package edu.wpi.first.wpilibj.examples.statespacedifferentialdrivesimulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constant.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.examples.statespacedifferentialdrivesimulation.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

import java.util.List;

public class RobotContainer {
    public final DriveSubsystem m_robotDrive = new DriveSubsystem();

    XboxController m_driverController =
        new XboxController(Constants.OIConstants.kDriverControllerPort);

        public RobotContainer() {
            configureButtonBindings();

            m_robotDrive.setDefaultCommand(
                new RunCommand(
                    () ->
                        m_robotDrive.arcadeDrive(
                            -m_driverController.getLeftY(), -m_driverController.getRightX()),
                        m_robotDrive));
        }

        private void configureButtonBindings() {
            new JoystickButton(m_driverController, Button.kRightBumper.value)
                .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
        }

        public DriveSubsystem getRobotDrive() {
            return m_robotDrive;
        }

        public void zeroAllOutputs() {
            m_robotDrive.tankDriveVolts(0, 0);
        }

        public Command getAutonomousCommand() {
            var autoVoltageConstraint = 
                new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                        Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                    Constants.DriveConstants.kDriveKinematics,
                    7);
            
            TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.DriveConstants.kDriveKinematics)
                    .addConstraint(autoVoltageConstraint);
            
            Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(1, 2, new Rotation2d(0)),
                    List.of(new Translation2d(2, 3), new Translation2d(3, 1)),
                    new Pose2d(4, 2, new Rotation2d(0)),
                    config);
            
            RamseteCommand ramseteCommand =
            new RamseteCommand(
                exampleTrajectory,
                m_robotDrive::getPose,
                new RamseteController(
                    Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(
                        Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                    Constants.DriveConstants.kDriveKinematics,
                    m_robotDrive::getWheelSpeeds,
                    new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                    new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                    m_robotDrive::tankDriveVolts,
                    m_robotDrive);
            
            m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

            return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));   
    }
}
