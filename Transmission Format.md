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
 (char)        (char)       (byte)       (byte)      (byte)      (byte)      (byte)
```
## Data Types
A= linear acceleration
G= orientation with respect to gravity (gyrometer)
M= orientation with respect to magnetic field (magnetometer)


## Vectors
The vectors are comprised of 3 strings (made of an unknown number of bytes) that represent 3 dimensions and they are X, Y and Z. The strings are deliniated by an @ symbol. The vectors come after the sensor ID. 

### Example
!A01234.12@123.34@156.76!A0... 
X= 1234.12
Y= 123.34
Z= 156.76

