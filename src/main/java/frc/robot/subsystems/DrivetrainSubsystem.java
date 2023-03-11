package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

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
        private Pose2d robotPose = new Pose2d();
        private Field2d field = new Field2d();
        private double simRotation = 0;
        private final Pose2d RED_ORIGIN = new Pose2d(new Translation2d(Constants.RED_ORIGIN_POS_X_METERS, Constants.RED_ORIGIN_POS_Y_METERS),Rotation2d.fromDegrees(Constants.RED_ORIGIN_ROTATION_DEG));

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private Rotation2d rotationTarget;
        private Translation2d translationTarget;
        private Rotation2d balanceTarget;
        private Pose2d balanceTargetOriginalPose;

        private PIDController yPID = new PIDController(TRANSLATION_PROPORTIONAL_COEFFICENT, TRANSLATION_INTEGRAL_COEFFICENT,
        TRANSLATION_DERIVATIVE_COEFFICENT);
        private PIDController xPID = new PIDController(TRANSLATION_PROPORTIONAL_COEFFICENT, TRANSLATION_INTEGRAL_COEFFICENT,
        TRANSLATION_DERIVATIVE_COEFFICENT);
        private PIDController rotationPID = new PIDController(ROTATION_PROPORTIONAL_COEFFICENT, ROTATION_INTEGRAL_COEFFICENT,
        ROTATION_DERIVATIVE_COEFFICENT);

        public DrivetrainSubsystem() {
                states = kinematics.toSwerveModuleStates(chassisSpeeds);
                SmartDashboard.putData("Field Sim", field);

                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
                Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();
                config.setDriveCurrentLimit(40);

                frontLeftModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                config,
                                Mk4SwerveModuleHelper.GearRatio.L3,
                                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                FRONT_LEFT_MODULE_STEER_MOTOR,
                                FRONT_LEFT_MODULE_STEER_ENCODER,
                                FRONT_LEFT_MODULE_STEER_OFFSET);

                frontRightModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                config,
                                Mk4SwerveModuleHelper.GearRatio.L3,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                backLeftModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                config,
                                Mk4SwerveModuleHelper.GearRatio.L3,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                BACK_LEFT_MODULE_STEER_OFFSET);

                backRightModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                config,
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

        public void zeroRotation() {
                resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
        }


        private SwerveModulePosition[] getModulePositions() {
                return new SwerveModulePosition[] {
                        new SwerveModulePosition(positionMeters[0], states[0].angle),
                        new SwerveModulePosition(positionMeters[1], states[1].angle),
                        new SwerveModulePosition(positionMeters[2], states[2].angle),
                        new SwerveModulePosition(positionMeters[3], states[3].angle)
                };
        }

        public void resetPose(Pose2d pose) {
                navx.reset();
                navx.setAngleAdjustment(pose.getRotation().getDegrees());
                if(Robot.isSimulation()) {
                        simRotation = pose.getRotation().getRadians();
                }

                odometry.resetPosition(
                        getGyroscopeRotation(),
                        getModulePositions(), 
                        pose
                );
        }

        private void updatePose() {
                if(Robot.isSimulation()) {
                        positionMeters[0] += states[0].speedMetersPerSecond * (Timer.getFPGATimestamp() - lastUpdateTime);
                        positionMeters[1] += states[1].speedMetersPerSecond * (Timer.getFPGATimestamp() - lastUpdateTime);
                        positionMeters[2] += states[2].speedMetersPerSecond * (Timer.getFPGATimestamp() - lastUpdateTime);
                        positionMeters[3] += states[3].speedMetersPerSecond * (Timer.getFPGATimestamp() - lastUpdateTime);
                        simRotation += kinematics.toChassisSpeeds(states).omegaRadiansPerSecond * (Timer.getFPGATimestamp() - lastUpdateTime);
                }else {
                        positionMeters[0] += frontLeftModule.getDriveVelocity() * (Timer.getFPGATimestamp() - lastUpdateTime); // (m / s) * delta t = m
                        positionMeters[1] += frontRightModule.getDriveVelocity() * (Timer.getFPGATimestamp() - lastUpdateTime);
                        positionMeters[2] += backLeftModule.getDriveVelocity() * (Timer.getFPGATimestamp() - lastUpdateTime);
                        positionMeters[3] += backRightModule.getDriveVelocity() * (Timer.getFPGATimestamp() - lastUpdateTime);
                }
                lastUpdateTime = Timer.getFPGATimestamp();

                robotPose = odometry.update(
                        getGyroscopeRotation(),
                        getModulePositions()
                );

                if(DriverStation.getAlliance() == Alliance.Red && robotPose != null){
                        field.setRobotPose(RED_ORIGIN.transformBy(new Transform2d(robotPose.getTranslation(), robotPose.getRotation())));
                } else {
                        field.setRobotPose(getPose());
                }
        }

        public SwerveDriveKinematics getKinematics() {
                return kinematics;
        }

        public Pose2d getPose() {
                return robotPose;
        }

        public Rotation2d getRotation() {
                return robotPose.getRotation();
        }

        public void setSwerveStates(SwerveModuleState[] states) {
                this.chassisSpeeds = kinematics.toChassisSpeeds(states);
        }

        private Rotation2d getGyroscopeRotation() {

                if(Robot.isSimulation()) {
                        return Rotation2d.fromRadians(simRotation);
                }

                if (navx.isMagnetometerCalibrated()) {
                        return Rotation2d.fromDegrees(navx.getFusedHeading());
                }

                return Rotation2d.fromDegrees(360.0 - navx.getYaw());
        }

        private Rotation2d getGyroscopePitch() {
                return Rotation2d.fromDegrees(navx.getPitch());
        }

        private Rotation2d getGyroscopeRoll() {
                return Rotation2d.fromDegrees(navx.getRoll());
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
                drive(0, 0, 0);
        }

        public Rotation2d getRotationTarget() {
                return rotationTarget;
        }

        public void setTranslationTarget(Translation2d translationTarget) {
                this.translationTarget = translationTarget;
                drive(0, 0, 0);
        }

        public Translation2d getTranslationTarget() {
                return translationTarget;
        }

        public void setBalanceTarget(Rotation2d balanceTarget) {
                this.balanceTarget = balanceTarget;
                balanceTargetOriginalPose = getPose();
                drive(0, 0, 0);
        }

        public void setBalanceTargetDefault() {
                setBalanceTarget(Rotation2d.fromDegrees(0));
        }

        public Rotation2d getBalanceTarget() {
                return balanceTarget;
        }

        @Override
        public void periodic() {
                if(rotationTarget != null && chassisSpeeds.omegaRadiansPerSecond == 0) {
                        chassisSpeeds.omegaRadiansPerSecond = rotationPID.calculate(getRotation().minus(rotationTarget).getRadians());
                }

                if(translationTarget != null && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0) {
                        chassisSpeeds.vxMetersPerSecond = xPID.calculate(getPose().getTranslation().minus(translationTarget).getX());
                        chassisSpeeds.vyMetersPerSecond = yPID.calculate(getPose().getTranslation().minus(translationTarget).getY());
                }

                if(balanceTarget != null ) {
                        if (Math.abs(getGyroscopePitch().getDegrees()) >= AUTO_BALANCE_TOLERANCE_DEGREES) {
                                chassisSpeeds.vyMetersPerSecond = yPID.calculate(-getGyroscopePitch().minus(balanceTarget).getDegrees());
                                chassisSpeeds.vxMetersPerSecond = xPID.calculate(-getGyroscopePitch().minus(balanceTarget).getDegrees());
                        } else if (Math.abs(getGyroscopeRoll().getDegrees()) >= AUTO_BALANCE_TOLERANCE_DEGREES) {
                                chassisSpeeds.vyMetersPerSecond = yPID.calculate(-getGyroscopeRoll().minus(balanceTarget).getDegrees());
                                chassisSpeeds.vxMetersPerSecond = xPID.calculate(-getGyroscopeRoll().minus(balanceTarget).getDegrees());
                        }

                        if(balanceTargetOriginalPose.getTranslation().getDistance(getPose().getTranslation()) > MAX_AUTO_BALANCE_TRANSLATION_METERS ) {
                                setBalanceTarget(null);
                                DriverStation.reportError("Auto balance disabled as measured translation has exceeded safety limit.", false);
                        }
                        
                }

                states = kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
                frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
                backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
                backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

                updatePose();
                SmartDashboard.putNumber("Gyro Roll", getGyroscopeRoll().getDegrees());
                SmartDashboard.putNumber("Gyro Pitch", getGyroscopePitch().getDegrees());
                SmartDashboard.putBoolean("Balance Target", getBalanceTarget() != null);
        }
}
