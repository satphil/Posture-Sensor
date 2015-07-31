# Command Format
```
    !            C           #         #           #           #           #

Start CMD    CMD type     Byte 0     Byte 1      Byte 2      Byte 3      Byte 4      ……
 (char)        (char)     (byte)     (byte)      (byte)      (byte)      (byte)
```

# Data Format
```
    !            A            0           #           #           #           #

Start Data    Data type    Sensor ID     Byte 0      Byte 1      Byte 2      Byte 3      ……
 (char)        (char)       (int)       (byte)      (byte)      (byte)      (byte)
```
## Data Types
### Acceleration
When the data type is A, the following bytes are acceleration in 3 dimensions. There are 3 bytes and they are X,Y and Z as ints.

### Orientation with respect to gravity (Gyrometer)
When the data type is G, the following bytes are orientation in 3 dimensions. There are 3 bytes and they are X,Y and Z as ints.

### Orientation with respect to magnetic field (Magnetometer)
When the data type is M, the following bytes are orientation in 3 dimensions. There are 3 bytes and they are X,Y and Z as ints.