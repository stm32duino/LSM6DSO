# LSM6DSO
Arduino library to support the LSM6DSO 3D accelerometer and 3D gyroscope

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    LSM6DSOSensor AccGyr(&dev_i2c);
    AccGyr.begin();
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LSM6DSOSensor AccGyr(&dev_spi, CS_PIN);
    AccGyr.begin();	
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

The access to the sensor values is done as explained below:  

  Read accelerometer and gyroscope.

    int32_t accelerometer[3];
    int32_t gyroscope[3];
    AccGyr.Get_X_Axes(accelerometer);  
    AccGyr.Get_G_Axes(gyroscope);

# Examples

There are several examples with the LSM6DSO library.
* LSM6DSO_HelloWorld: This application provides a simple example of usage of the LSM6DSO 
IMU 6-axis. It shows how to display on a hyperterminal the values of the sensor.
* LSM6DSO_6DOrientation: This application shows how to use the LSM6DSO accelerometer 
to find out the 6D orientation and display data on a hyperterminal.
* LSM6DSO_FreeFallDetection: This application shows how to detect the free fall event using the 
LSM6DSO accelerometer.
* LSM6DSO_Pedometer: This application shows how to use the LSM6DSO accelerometer 
to count steps.
* LSM6DSO_SingleTap: This application shows how to detect the single tap event using the 
LSM6DSO accelerometer.
* LSM6DSO_DoubleTap: This application shows how to detect the double tap event using the 
LSM6DSO accelerometer.
* LSM6DSO_TiltDetection: This application shows how to detect the tilt event using the 
LSM6DSO accelerometer.
* LSM6DSO_WakeUpDetection: This application shows how to detect the wake-up event using the 
LSM6DSO accelerometer.

## Documentation

You can find the source files at  
https://github.com/stm32duino/LSM6DSO

The LSM6DSO datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dso.html
