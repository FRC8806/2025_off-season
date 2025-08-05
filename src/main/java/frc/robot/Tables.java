package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;

import java.util.HashMap;
import java.util.Map;

public class Tables {
  private NetworkTable table;
  private Map<String, Sendable> tablesToData = new HashMap<>();
  private static boolean m_reported = false; // NOPMD redundant field initializer

  public Tables(String tableName) {
    this.table = NetworkTableInstance.getDefault().getTable(tableName);
    tablesToData.clear();
  }

  public NetworkTableEntry getEntry(String key) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_SmartDashboard, tInstances.kSmartDashboard_Instance);
      m_reported = true;
    }
    return this.table.getEntry(key);
  }

  public String getString(String key, String defaultValue) {
    return getEntry(key).getString(defaultValue);
  }

  public Double getDouble(String key, double defaultValue) {
    return getEntry(key).getDouble(defaultValue);
  }

  

  public Double getX() {
    return Double.parseDouble(this.getString("x", "") != null ? this.getString("x", "") : "0");
  }

  public Double getY() {
    return Double.parseDouble(this.getString("y", "") != null ? this.getString("y", "") : "0");
  }

  public Double getZ() {
    return Double.parseDouble(this.getString("z", "") != null ? this.getString("z", "") : "0");
  }

}
