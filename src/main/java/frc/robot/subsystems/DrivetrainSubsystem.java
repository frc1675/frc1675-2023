package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
        public static final double MAX_VOLTAGE = 12;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                        SdsModuleConfigurations.MK4_L3.getDriveReduction() *
                        SdsModuleConfigurations.MK4_L3.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200);

        private final SwerveModule frontLeftModule;
        private final SwerveModule frontRightModule;
        private final SwerveModule backLeftModule;
        private final SwerveModule backRightModule;

        private NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");
        private DoubleTopic topic = table.getDoubleTopic("Gyro Rotation");
        private DoublePublisher rotation = topic.publish();

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                frontLeftModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                Mk4SwerveModuleHelper.GearRatio.L3,
                                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                FRONT_LEFT_MODULE_STEER_MOTOR,
                                FRONT_LEFT_MODULE_STEER_ENCODER,
                                FRONT_LEFT_MODULE_STEER_OFFSET);

                frontRightModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4SwerveModuleHelper.GearRatio.L3,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                backLeftModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4SwerveModuleHelper.GearRatio.L3,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                BACK_LEFT_MODULE_STEER_OFFSET);

                backRightModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4SwerveModuleHelper.GearRatio.L3,
                                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR,
                                BACK_RIGHT_MODULE_STEER_ENCODER,
                                BACK_RIGHT_MODULE_STEER_OFFSET);
        }

        public void zeroGyroscope() {
                navx.zeroYaw();
        }

        public Rotation2d getGyroscopeRotation() {

                if (navx.isMagnetometerCalibrated()) {
                        return Rotation2d.fromDegrees(navx.getFusedHeading());
                }

                return Rotation2d.fromDegrees(360.0 - navx.getYaw());
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                this.chassisSpeeds = chassisSpeeds;
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

                rotation.set(getGyroscopeRotation().getDegrees());
        }

        public void rotate(double radiansPerSecond) {
                drive(new ChassisSpeeds(0, 0, radiansPerSecond));
        }
}
