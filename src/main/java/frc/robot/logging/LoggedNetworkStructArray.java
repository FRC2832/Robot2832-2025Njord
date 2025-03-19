package frc.robot.logging;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayEntry;
import edu.wpi.first.util.struct.Struct;

public class LoggedNetworkStructArray<T> extends LoggedNetworkInput {
    private String key;
    private StructArrayEntry<T> entry;
    private Struct<T> struct;
    private T[] value, defaultValue = null;

    public LoggedNetworkStructArray(String key, Struct<T> struct){
        this.key = key;
        this.entry = NetworkTableInstance.getDefault().getStructArrayTopic(key, struct).getEntry(defaultValue);
        this.value = defaultValue;
        this.struct = struct;
        Logger.registerDashboardInput(this);
    }

    public LoggedNetworkStructArray(String key, Struct<T> struct, T[] defaultValue){
        this(key, struct);
        setDefault(defaultValue);
        this.value = defaultValue;
    }

    public void setDefault(T[] defaultValue){
        this.defaultValue = defaultValue;
    }

    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(removeSlash(key), struct, value);
        }

        public void fromLog(LogTable table) {
            value = table.get(removeSlash(key), struct, defaultValue);
        }
    };

    @Override
    public void periodic() {
        if (!Logger.hasReplaySource()) {
            value = entry.get(defaultValue);
          }
          Logger.processInputs(prefix, inputs);
    }

}
