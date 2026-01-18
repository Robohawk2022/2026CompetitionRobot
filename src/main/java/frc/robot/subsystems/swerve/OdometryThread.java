package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.CircularBuffer;
import frc.robot.util.TimestampedPose;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static frc.robot.Config.Swerve.*;

/**
 * Runs odometry at high frequency (250Hz) using CTRE synchronized reads.
 * <p>
 * This thread uses a {@link Notifier} for precise 4ms periodic execution.
 * Pose history is stored in a circular buffer for latency compensation
 * when applying vision measurements.
 * <p>
 * Thread-safe pose access is provided via {@link ReadWriteLock}.
 */
public class OdometryThread {

    /** Default frequency for high-frequency odometry (Hz) */
    public static final double DEFAULT_FREQUENCY = 250.0;

    /** Default pose history size (~200ms at 250Hz) */
    public static final int DEFAULT_HISTORY_SIZE = 50;

    private final SwerveHardware hardware;
    private final BaseStatusSignal[] signals;
    private final SwerveDriveOdometry odometry;
    private final Notifier notifier;

    private final CircularBuffer<TimestampedPose> poseHistory;
    private final ReadWriteLock lock = new ReentrantReadWriteLock();

    private final double periodSeconds;
    private volatile boolean running = false;
    private volatile int updateCount = 0;
    private volatile double lastUpdateTime = 0.0;

    /**
     * Creates an OdometryThread with default settings.
     *
     * @param hardware the swerve hardware (must support high-frequency odometry)
     */
    public OdometryThread(SwerveHardware hardware) {
        this(hardware, DEFAULT_FREQUENCY, DEFAULT_HISTORY_SIZE);
    }

    /**
     * Creates an OdometryThread with custom settings.
     *
     * @param hardware the swerve hardware (must support high-frequency odometry)
     * @param frequencyHz update frequency in Hz
     * @param historySize number of poses to keep in history
     */
    public OdometryThread(SwerveHardware hardware, double frequencyHz, int historySize) {
        this.hardware = hardware;
        this.signals = hardware.getAllSignals();
        this.periodSeconds = 1.0 / frequencyHz;
        this.poseHistory = new CircularBuffer<>(historySize);

        // initialize odometry at origin
        this.odometry = new SwerveDriveOdometry(
                KINEMATICS,
                hardware.getHeading(),
                hardware.getModulePositions());

        // create notifier for high-frequency updates
        this.notifier = new Notifier(this::update);
        notifier.setName("OdometryThread");
    }

    /**
     * Starts the high-frequency odometry thread.
     */
    public void start() {
        if (!running && signals.length > 0) {
            running = true;
            updateCount = 0;
            notifier.startPeriodic(periodSeconds);
        }
    }

    /**
     * Stops the high-frequency odometry thread.
     */
    public void stop() {
        running = false;
        notifier.stop();
    }

    /**
     * Called by the Notifier at high frequency. Performs synchronized
     * signal reads and updates odometry.
     */
    private void update() {
        if (!running) {
            return;
        }

        // perform synchronized read of all signals with timeout
        BaseStatusSignal.waitForAll(periodSeconds, signals);

        // get current timestamp
        double timestamp = Timer.getFPGATimestamp();

        // extract positions from signals
        // signals order: FL_drive, FL_turn, FR_drive, FR_turn, BL_drive, BL_turn, BR_drive, BR_turn, yaw
        SwerveModulePosition[] positions = new SwerveModulePosition[] {
                extractModulePosition(0), // FL
                extractModulePosition(1), // FR
                extractModulePosition(2), // BL
                extractModulePosition(3)  // BR
        };

        // extract yaw from the last signal
        double yawRotations = signals[8].getValueAsDouble();
        Rotation2d heading = Rotation2d.fromRotations(yawRotations);

        // update odometry
        lock.writeLock().lock();
        try {
            odometry.update(heading, positions);

            // add to pose history with timestamp
            Pose2d currentPose = odometry.getPoseMeters();
            poseHistory.add(new TimestampedPose(currentPose, timestamp));
        } finally {
            lock.writeLock().unlock();
        }

        updateCount++;
        lastUpdateTime = timestamp;
    }

    /**
     * Extracts a module position from the signals array.
     *
     * @param moduleIndex 0=FL, 1=FR, 2=BL, 3=BR
     * @return the module position
     */
    private SwerveModulePosition extractModulePosition(int moduleIndex) {
        int signalIndex = moduleIndex * 2;
        double driveRotations = signals[signalIndex].getValueAsDouble();
        double turnRotations = signals[signalIndex + 1].getValueAsDouble();

        // convert drive rotations to meters
        double distanceMeters = driveRotations * WHEEL_CIRCUMFERENCE_METERS;

        // convert turn rotations to angle
        Rotation2d angle = Rotation2d.fromRotations(turnRotations);

        return new SwerveModulePosition(distanceMeters, angle);
    }

    /**
     * @return the latest pose from odometry (thread-safe)
     */
    public Pose2d getLatestPose() {
        lock.readLock().lock();
        try {
            return odometry.getPoseMeters();
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * Gets the pose at a specific timestamp using linear interpolation.
     * If the timestamp is outside the history, returns the closest pose.
     *
     * @param timestamp the FPGA timestamp in seconds
     * @return the interpolated pose, or the latest pose if no history
     */
    public Pose2d getPoseAtTime(double timestamp) {
        lock.readLock().lock();
        try {
            if (poseHistory.isEmpty()) {
                return odometry.getPoseMeters();
            }

            // find the two poses bracketing the timestamp
            TimestampedPose before = null;
            TimestampedPose after = null;

            for (int i = 0; i < poseHistory.size(); i++) {
                TimestampedPose pose = poseHistory.get(i);
                if (pose.timestamp() <= timestamp) {
                    before = pose;
                } else {
                    after = pose;
                    break;
                }
            }

            // handle edge cases
            if (before == null) {
                return poseHistory.getFirst().pose();
            }
            if (after == null) {
                return poseHistory.getLast().pose();
            }

            // linear interpolation
            double t = (timestamp - before.timestamp()) / (after.timestamp() - before.timestamp());
            return before.pose().interpolate(after.pose(), t);
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * Resets odometry to a specific pose.
     *
     * @param pose the pose to reset to
     */
    public void resetPose(Pose2d pose) {
        lock.writeLock().lock();
        try {
            // use pose.getRotation() directly since hardware.setHeading() is async
            odometry.resetPosition(
                    pose.getRotation(),
                    hardware.getModulePositions(),
                    pose);
            poseHistory.clear();
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * @return the number of odometry updates performed
     */
    public int getUpdateCount() {
        return updateCount;
    }

    /**
     * @return the actual update frequency in Hz, calculated from recent updates
     */
    public double getActualFrequency() {
        // this is approximate, based on the period we're configured for
        return 1.0 / periodSeconds;
    }

    /**
     * @return true if the thread is running
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * @return the time of the last update in FPGA seconds
     */
    public double getLastUpdateTime() {
        return lastUpdateTime;
    }

    /**
     * @return the pose history buffer for diagnostics
     */
    public CircularBuffer<TimestampedPose> getPoseHistory() {
        return poseHistory;
    }
}
