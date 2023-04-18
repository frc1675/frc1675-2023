package frc.robot.subsystems;

import static frc.robot.Constants.AUTO_BALANCE_TOLERANCE_DEGREES;
import static frc.robot.Constants.MAX_AUTO_BALANCE_TRANSLATION_METERS;
import static frc.robot.Constants.ROTATION_DERIVATIVE_COEFFICENT;
import static frc.robot.Constants.ROTATION_INTEGRAL_COEFFICENT;
import static frc.robot.Constants.ROTATION_PROPORTIONAL_COEFFICENT;
import static frc.robot.Constants.TRANSLATION_DERIVATIVE_COEFFICENT;
import static frc.robot.Constants.TRANSLATION_INTEGRAL_COEFFICENT;
import static frc.robot.Constants.TRANSLATION_PROPORTIONAL_COEFFICENT;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveKinematics2;
import swervelib.parser.SwerveParser;

public class DrivetrainSubsystem extends SubsystemBase {
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.4864;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 15.66;
        private final SwerveDrive swerveDrive;

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private Rotation2d rotationTarget;
        private Translation2d translationTarget;
        private Rotation2d balanceTarget;
        private Pose2d balanceTargetOriginalPose;

        private PIDController yPID = new PIDController(TRANSLATION_PROPORTIONAL_COEFFICENT, TRANSLATION_INTEGRAL_COEFFICENT, TRANSLATION_DERIVATIVE_COEFFICENT);
        private PIDController xPID = new PIDController(TRANSLATION_PROPORTIONAL_COEFFICENT, TRANSLATION_INTEGRAL_COEFFICENT, TRANSLATION_DERIVATIVE_COEFFICENT);
        private PIDController rotationPID = new PIDController(ROTATION_PROPORTIONAL_COEFFICENT, ROTATION_INTEGRAL_COEFFICENT, ROTATION_DERIVATIVE_COEFFICENT);

        public DrivetrainSubsystem() {
                try {
                        File f = new File(Filesystem.getDeployDirectory(), "swerve");
                        SwerveParser s = new SwerveParser(f);
                        swerveDrive = s.createSwerveDrive();
                } catch (IOException e) {
                        throw new RuntimeException("Swerve parser failed");
                }
        }

        public void setSwerveStates(SwerveModuleState[] states) {
                this.chassisSpeeds = swerveDrive.kinematics.toChassisSpeeds(states);
        }

        public void setSpeeds(ChassisSpeeds chassisSpeeds) {
                this.chassisSpeeds = chassisSpeeds;
        }

        public void setSpeeds(double x, double y, double rotation) {
                this.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getGyroscopeYaw()));
        }

        public Rotation2d getGyroscopeRoll() {
                return swerveDrive.getRoll();
        }

        public Rotation2d getGyroscopePitch() {
                return swerveDrive.getPitch();
        }

        public Rotation2d getGyroscopeYaw() {
                return swerveDrive.getYaw();
        }

        public void zeroGyroscope() {
                swerveDrive.zeroGyro();
        }

        public double getCurrentSpeed() {
                return Math.abs(Math.sqrt(
                                (chassisSpeeds.vxMetersPerSecond * chassisSpeeds.vxMetersPerSecond)
                                +
                                (chassisSpeeds.vyMetersPerSecond * chassisSpeeds.vyMetersPerSecond)
                        ));
        }

        public Pose2d getPose() {
                return swerveDrive.getPose();
        }

        public void resetPose(Pose2d pose) {
                swerveDrive.resetOdometry(pose);
        }

        public SwerveKinematics2 getKinematics() {
                return swerveDrive.kinematics;
        }

        public void setRotationTarget(Rotation2d rotationTarget) {
                this.rotationTarget = rotationTarget;
                setSpeeds(0, 0, 0);
        }

        public Rotation2d getRotationTarget() {
                return rotationTarget;
        }

        public void setTranslationTarget(Translation2d translationTarget) {
                this.translationTarget = translationTarget;
                setSpeeds(0, 0, 0);
        }

        public Translation2d getTranslationTarget() {
                return translationTarget;
        }

        public void setBalanceTarget(Rotation2d balanceTarget) {
                this.balanceTarget = balanceTarget;
                balanceTargetOriginalPose = getPose();
                setSpeeds(0, 0, 0);
        }

        public void setBalanceTargetDefault() {
                setBalanceTarget(Rotation2d.fromDegrees(0));
        }

        public Rotation2d getBalanceTarget() {
                return balanceTarget;
        }

        @Override
        public void periodic() {
                if (rotationTarget != null && chassisSpeeds.omegaRadiansPerSecond == 0) {
                        chassisSpeeds.omegaRadiansPerSecond = rotationPID
                                        .calculate(getGyroscopeYaw().minus(rotationTarget).getRadians());
                }

                if (translationTarget != null && chassisSpeeds.vxMetersPerSecond == 0
                                && chassisSpeeds.vyMetersPerSecond == 0) {
                        chassisSpeeds.vxMetersPerSecond = xPID
                                        .calculate(getPose().getTranslation().minus(translationTarget).getX());
                        chassisSpeeds.vyMetersPerSecond = yPID
                                        .calculate(getPose().getTranslation().minus(translationTarget).getY());
                }

                if (balanceTarget != null) {

                        if (Math.abs(getGyroscopePitch().getDegrees()) >= AUTO_BALANCE_TOLERANCE_DEGREES
                                        && Math.abs(getGyroscopePitch().getDegrees()) > Math
                                                        .abs(getGyroscopeRoll().getDegrees())) {
                                chassisSpeeds.vyMetersPerSecond = yPID
                                                .calculate(-getGyroscopePitch().minus(balanceTarget).getDegrees());
                        } else if (Math.abs(getGyroscopeRoll().getDegrees()) >= AUTO_BALANCE_TOLERANCE_DEGREES
                                        && Math.abs(getGyroscopeRoll().getDegrees()) > Math
                                                        .abs(getGyroscopePitch().getDegrees())) {
                                chassisSpeeds.vxMetersPerSecond = xPID
                                                .calculate(-getGyroscopeRoll().minus(balanceTarget).getDegrees());
                        } else {
                                chassisSpeeds.vyMetersPerSecond = 0;
                                chassisSpeeds.vxMetersPerSecond = 0;
                                xPID.reset();
                                yPID.reset();
                        }

                        if (balanceTargetOriginalPose.getTranslation().getDistance(
                                        getPose().getTranslation()) > MAX_AUTO_BALANCE_TRANSLATION_METERS) {
                                setBalanceTarget(null);
                                DriverStation.reportError(
                                                "Auto balance disabled as measured translation has exceeded safety limit.",
                                                false);
                        }

                }
                swerveDrive.setChassisSpeeds(chassisSpeeds);
                swerveDrive.updateOdometry();
        }
}
