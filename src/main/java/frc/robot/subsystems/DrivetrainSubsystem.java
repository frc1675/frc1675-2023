package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
        public static final double MAX_VOLTAGE = 12;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                        SdsModuleConfigurations.MK4_L3.getDriveReduction() *
                        SdsModuleConfigurations.MK4_L3.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200);

        private final SwerveModule frontLeftModule;
        private final SwerveModule frontRightModule;
        private final SwerveModule backLeftModule;
        private final SwerveModule backRightModule;

        private NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");

        private BooleanTopic BalanceTargetTopic = table.getBooleanTopic("Has Balance Target");
        private BooleanPublisher hasBalanceTarget = BalanceTargetTopic.publish();

        private DoubleTopic rotationTopic = table.getDoubleTopic("Gyro Rotation");
        private DoublePublisher rotation = rotationTopic.publish();

        private DoubleTopic pitchTopic = table.getDoubleTopic("Gyro Pitch");
        private DoublePublisher pitch = pitchTopic.publish();

        private DoubleTopic rollTopic = table.getDoubleTopic("Gyro Roll");
        private DoublePublisher roll = rollTopic.publish();

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private boolean updateRotationTarget = true;
        private boolean forceRotationTarget = false;
        private Rotation2d rotationTarget = new Rotation2d(0);

        private Rotation2d balanceTarget = null;

        private PIDController yPID = new PIDController(Constants.PROPORTIONAL_COEFFICENT, Constants.INTEGRAL_COEFFICENT,
                        Constants.DERIVATIVE_COEFFICENT);
        private PIDController xPID = new PIDController(Constants.PROPORTIONAL_COEFFICENT, Constants.INTEGRAL_COEFFICENT,
                        Constants.DERIVATIVE_COEFFICENT);
        private PIDController rotationPID = new PIDController(Constants.PROPORTIONAL_COEFFICENT,
                        Constants.INTEGRAL_COEFFICENT,
                        Constants.DERIVATIVE_COEFFICENT);

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                frontLeftModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                Mk4SwerveModuleHelper.GearRatio.L3,
                                Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                                Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                                Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

                frontRightModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4SwerveModuleHelper.GearRatio.L3,
                                Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                                Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

                backLeftModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4SwerveModuleHelper.GearRatio.L3,
                                Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                                Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                                Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                                Constants.BACK_LEFT_MODULE_STEER_OFFSET);

                backRightModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4SwerveModuleHelper.GearRatio.L3,
                                Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                                Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                                Constants.BACK_RIGHT_MODULE_STEER_OFFSET);
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

        public Rotation2d getGyroscopePitch() {
                return Rotation2d.fromDegrees(navx.getPitch());
        }

        public Rotation2d getGyroscopeRoll() {
                return Rotation2d.fromDegrees(navx.getRoll());
        }

        public void toggleBalanceTarget() {
                if (balanceTarget == null) {
                        balanceTarget = Rotation2d.fromDegrees(Constants.AUTO_BALANCE_TARGET_DEGREES);
                } else {
                        balanceTarget = null;
                }
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                this.chassisSpeeds = chassisSpeeds;
        }

        /* Convience method, calls drive. Uses field relative controls. */
        public void drive(double x, double y, double rotation) {
                drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getGyroscopeRotation()));
        }

        public void setRotationTarget(Rotation2d rotationTarget) {
                this.rotationTarget = rotationTarget;
                forceRotationTarget = true;
        }

        public void unsetRotationTarget() {
                forceRotationTarget = false;
        }

        @Override
        public void periodic() {
                if (chassisSpeeds.omegaRadiansPerSecond == 0
                                && (chassisSpeeds.vxMetersPerSecond != 0 || chassisSpeeds.vyMetersPerSecond != 0)) {
                        chassisSpeeds.omegaRadiansPerSecond = -rotationPID
                                        .calculate(getGyroscopeRotation().minus(rotationTarget).getRadians());
                        if (updateRotationTarget && !forceRotationTarget) {
                                rotationTarget = getGyroscopeRotation();
                                updateRotationTarget = false;
                        }
                } else {
                        updateRotationTarget = true;
                }

                if (balanceTarget != null) {
                        if (Math.abs(getGyroscopePitch().getDegrees()) >= Constants.AUTO_BALANCE_TOLERANCE_DEGREES) {
                                chassisSpeeds.vyMetersPerSecond = MAX_VELOCITY_METERS_PER_SECOND * 0.25;
                        } else if (Math.abs(
                                        getGyroscopeRoll().getDegrees()) >= Constants.AUTO_BALANCE_TOLERANCE_DEGREES) {
                                chassisSpeeds.vxMetersPerSecond = MAX_VELOCITY_METERS_PER_SECOND * 0.25;
                        }
                }

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
                pitch.set(getGyroscopePitch().getDegrees());
                roll.set(getGyroscopeRoll().getDegrees());
                hasBalanceTarget.set(balanceTarget != null);
        }
}