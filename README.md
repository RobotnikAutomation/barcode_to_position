# barcode_to_position

The barcode_to_position package, based on RComponent structure. This node allows to get the real position of the robot from a barcode scanner.

## 1 barcode_to_position_node

Node that launches an instance of the BarcodeToPosition class.

### 1.1 Parameters

* ~desired_freq (float, default: 10.0)
   Desired frequency of the node.
* ~modbus_io_sub_name (string, default: robotnik_modbus_io/registers)
   Name of the topic used for reading the barcode scanner. Using registers 0 and 1 for the front barcode scanner, and 2 and 3 for the rear barcode scanner.
* ~real_pos_yaml_path (string, default: barcode_to_real_positions.yaml)
   Yaml used for configuring the equivalence between the barcodes and the real positions.
   
### 1.2 Subscribed Topics

* robotnik_modbus_io/registers (robotnik_msgs/Registers)
  Topic used for reading the barcode scanner. Using registers 0 and 1 for the front barcode scanner, and 2 and 3 for the rear barcode scanner.

### 1.3 Published Topics

* ~status (std_msgs/String)
  Node information.
* ~status_stamped (robotnik_msgs/StringStamped)
  Node information with timestamp.
* ~barcode_scan_position (nav_msgs/Odometry)
  Real position of the base_footprint relative to robot_odom.
* ~front_table_id (std_msgs/Int16)
  Id of the table that the front barcode scanner is reading (0 if no table).
* ~rear_table_id (std_msgs/Int16)
  Id of the table that the rear barcode scanner is reading (0 if no table).

### 1.4 Bringup

```bash
roslaunch barcode_to_position barcode_to_position.launch
``` 

### 1.5 Yaml file

* padding (float)
  Minimum padding between the barcodes of each table for avoiding overlaps. Is checked at the start of the node.
* initial_yaw (float)
  Parameter to define the relative orientation of the barcodes on the line.
* front_barcodes/barcode_pos (float[2])
  Range for each barcode on a table
* front_barcodes/initial_real_pos (float)
  The real position on the beggining of that barcode. Only used on front barcode reader.
* front_barcodes/table_id (int)
  The ID of that table.

```yaml
padding: 0.15
initial_yaw: 0.0

front_barcodes:
  -
    barcode_pos: [0.0, 1.0]
    initial_real_pos: 0.0 # Offset to apply to the barcode position (meters)
    table_id: 24
  -
    barcode_pos: [1.2, 2.2]
    initial_real_pos: 1.5
    table_id: 25
  -
    barcode_pos: [2.4, 3.4]
    initial_real_pos: 3.0
    table_id: 26

rear_barcodes: # No initial_real_pos required due it is not being used for localization
  -
    barcode_pos: [21.0, 23.2]
    table_id: 123
```