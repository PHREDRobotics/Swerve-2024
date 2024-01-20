package edu.wpi.first.wpilibj.examples.statespacedifferentialdrivesimulation;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;

        public static final int[] kLeftEncoderPorts = new int[] {0, 1};
        public static final int[] kRightEncoderPorts = new int[] {2, 3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = 
            new DifferentialDriveKinematics(kTrackwidthMeters);
        
        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse = 
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        
        public static final boolean kGyroReversed = true;

        public static final double kvVoltSecondsPerRadian = 1.5;
        public static final double kaVoltSecondsSquaredPerRadian = 0.3;

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
            LinearSystemId.identifyDrivetrainSystem(
                kvVoltSecondsPerMeter,
                kaVoltSecondsSquaredPerRadian,
                kvVoltSecondsPerRadian,
                kaVoltSecondsSquaredPerRadian);

        public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
        public static final double kDriveGearing = 8;

        public static final double kPDriveVel = 8.5;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
