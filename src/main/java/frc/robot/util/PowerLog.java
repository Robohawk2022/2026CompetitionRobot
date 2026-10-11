package frc.robot.util;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Writes the roboRIO's own view of power and CAN health straight to the
 * .wpilog every loop. These are the numbers you need to diagnose a
 * brownout and none of them were being recorded anywhere before.
 *
 * <p>Written directly with DataLog entries rather than via
 * NetworkTables, so they cost nothing on the dashboard and sample every
 * 20 ms regardless of NT publish rate.</p>
 *
 * <p>Usage: {@code PowerLog.start()} once after
 * {@code DataLogManager.start()}, then {@code PowerLog.update()} from
 * {@code robotPeriodic()}.</p>
 */
public final class PowerLog {

    static DoubleLogEntry batteryVoltage;
    static DoubleLogEntry inputVoltage;
    static DoubleLogEntry inputCurrent;
    static BooleanLogEntry brownedOut;
    static IntegerLogEntry brownoutCount;
    static DoubleLogEntry canUtilization;
    static IntegerLogEntry canBusOff;
    static IntegerLogEntry canTxFull;
    static IntegerLogEntry canRxErrors;
    static IntegerLogEntry canTxErrors;

    static int brownouts = 0;
    static boolean wasBrownedOut = false;

    private PowerLog() {
    }

    public static void start() {
        DataLog log = DataLogManager.getLog();
        batteryVoltage = new DoubleLogEntry(log, "/power/batteryVolts");
        inputVoltage = new DoubleLogEntry(log, "/power/rioInputVolts");
        inputCurrent = new DoubleLogEntry(log, "/power/rioInputAmps");
        brownedOut = new BooleanLogEntry(log, "/power/brownedOut");
        brownoutCount = new IntegerLogEntry(log, "/power/brownoutCount");
        canUtilization = new DoubleLogEntry(log, "/can/utilizationPct");
        canBusOff = new IntegerLogEntry(log, "/can/busOffCount");
        canTxFull = new IntegerLogEntry(log, "/can/txFullCount");
        canRxErrors = new IntegerLogEntry(log, "/can/rxErrorCount");
        canTxErrors = new IntegerLogEntry(log, "/can/txErrorCount");
    }

    public static void update() {
        if (batteryVoltage == null) {
            return;
        }

        batteryVoltage.append(RobotController.getBatteryVoltage());
        inputVoltage.append(RobotController.getInputVoltage());
        inputCurrent.append(RobotController.getInputCurrent());

        boolean bo = RobotController.isBrownedOut();
        brownedOut.append(bo);
        if (bo && !wasBrownedOut) {
            brownouts++;
            Util.log("[power] BROWNOUT #%d at %.2f V", brownouts, RobotController.getBatteryVoltage());
        }
        wasBrownedOut = bo;
        brownoutCount.append(brownouts);

        var can = RobotController.getCANStatus();
        canUtilization.append(can.percentBusUtilization * 100.0);
        canBusOff.append(can.busOffCount);
        canTxFull.append(can.txFullCount);
        canRxErrors.append(can.receiveErrorCount);
        canTxErrors.append(can.transmitErrorCount);
    }
}
