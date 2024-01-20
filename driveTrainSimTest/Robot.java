package edu.wpi.first.wpilibj.examples.statespacedifferentialdrivesimulation;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.spi.first.wpilibj.simulation.RobotRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;

public class Robot extends TimedRobot {
    private RobotContainer m_robotContainer;
    
    private final Field2d m_field = new Field2d();

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        m_trajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
                new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Push the trajectory to Field2d.
    m_field.getObject("traj").setTrajectory(m_trajectory);
    }
    
    @Override
    public void simulationPeriodic() {
        double drawCurrent = m_robotContainer.getRobotDrive().getDrawnCurrentAmps();
        double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
        RoboRioSim.setVInVoltage(loadedVoltage);
        
        m_field.setRobotPose(RobotContainer.m_RobotDrive::getPose);
    }
    
    @Override
    public void autonomousInit() {
        m_robotContainer.getAutonomousCommand().schedule();
    }
    
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.zeroAllOutputs();
    }
}
