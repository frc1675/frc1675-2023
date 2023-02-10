package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DERIVATIVE_COEFFICENT;
import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.INTEGRAL_COEFFICENT;
import static frc.robot.Constants.PROPORTIONAL_COEFFICENT;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
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

        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
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

        private double[] positionMeters = new double[4];
        private double lastUpdateTime = 0;

        private SwerveModuleState[] states;
        private SwerveDriveOdometry odometry;
        private Pose2d robotPose;

        private NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");
        private DoubleTopic gyroTopic = table.getDoubleTopic("Gyro Rotation");
        private DoublePublisher gyroReading = gyroTopic.publish();

        private BooleanTopic rotationTargetTopic = table.getBooleanTopic("Has Rotation Target");
        private BooleanPublisher hasRotationTarget = rotationTargetTopic.publish(); 

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private boolean updateRotationTarget = true;
        private boolean forceRotationTarget = false;
        private Rotation2d rotationTarget = new Rotation2d(0);

        private PIDController yPID = new PIDController(PROPORTIONAL_COEFFICENT, INTEGRAL_COEFFICENT,
                        DERIVATIVE_COEFFICENT);
        private PIDController xPID = new PIDController(PROPORTIONAL_COEFFICENT, INTEGRAL_COEFFICENT,
                        DERIVATIVE_COEFFICENT);
        private PIDController rotationPID = new PIDController(PROPORTIONAL_COEFFICENT, INTEGRAL_COEFFICENT,
                        DERIVATIVE_COEFFICENT);

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
                
                odometry = new SwerveDriveOdometry(
                        kinematics,
                        getGyroscopeRotation(),
                        getModulePositions()
                );
        }

        public void zeroGyroscope() {
                navx.zeroYaw();
        }

        private SwerveModulePosition[] getModulePositions() {
                return new SwerveModulePosition[] {
                        new SwerveModulePosition(positionMeters[0], states[0].angle),
                        new SwerveModulePosition(positionMeters[1], states[1].angle),
                        new SwerveModulePosition(positionMeters[2], states[2].angle),
                        new SwerveModulePosition(positionMeters[3], states[3].angle)
                };
        }

        public void resetGyroscope(Pose2d pose) {
                odometry.resetPosition(
                        getGyroscopeRotation(),
                        getModulePositions(), 
                        pose
                );
        }

        public SwerveDriveKinematics getKinematics() {
                return kinematics;
        }

        public Pose2d getPose() {
                return robotPose;
        }

        public void setSwerveStates(SwerveModuleState[] states) {
                this.states = states;
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

        /* Convience method, calls drive. Uses field relative controls. */
        public void drive(double x, double y, double rotation) {
                drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getGyroscopeRotation()));
        }

        public void enableRotationTarget(Rotation2d rotationTarget) {
                this.rotationTarget = rotationTarget;
                forceRotationTarget = true;
        }

        public void disableRotationTarget() {
                forceRotationTarget = false;
        }

        public boolean getRotationTargetEnabled() {
                return forceRotationTarget;
        }

        @Override
        public void periodic() {
                if (chassisSpeeds.omegaRadiansPerSecond == 0 && (chassisSpeeds.vxMetersPerSecond != 0 || chassisSpeeds.vyMetersPerSecond != 0)) {
                        chassisSpeeds.omegaRadiansPerSecond = rotationPID.calculate(getGyroscopeRotation().minus(rotationTarget).getRadians());
                        if (updateRotationTarget && !forceRotationTarget) {
                                rotationTarget = getGyroscopeRotation();
                                updateRotationTarget = false;
                        }
                } else {
                        updateRotationTarget = true;
                }
                
                positionMeters[0] += frontLeftModule.getDriveVelocity() * Timer.getFPGATimestamp() - lastUpdateTime; // (m / s) * delta t = m
                positionMeters[1] += frontRightModule.getDriveVelocity() * Timer.getFPGATimestamp() - lastUpdateTime;
                positionMeters[2] += backLeftModule.getDriveVelocity() * Timer.getFPGATimestamp() - lastUpdateTime;
                positionMeters[3] += backRightModule.getDriveVelocity() * Timer.getFPGATimestamp() - lastUpdateTime;
                lastUpdateTime = Timer.getFPGATimestamp();

                robotPose = odometry.update(
                        getGyroscopeRotation(),
                        getModulePositions()
                );

                if(chassisSpeeds.omegaRadiansPerSecond + chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vxMetersPerSecond == 0) {
                        states = kinematics.toSwerveModuleStates(chassisSpeeds);
                }
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

                gyroReading.set(getGyroscopeRotation().getDegrees());
                hasRotationTarget.set(forceRotationTarget);
        }
}
