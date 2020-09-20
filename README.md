# PSoC_MPU6050
Please replace I2C_1 with whatever naming of your I2C module for your PSoC project, Haven't find a way to generalise the method so it takes would work on any PSoC project, since PSoC generate the I2C modules from the TopDesign. 

This library has part of the I2C dev lib and MPU6050 lib, both by Jeff Rowberg. This libaray is based on MPU6050 and is not as generalised as the original C++ lib. What this libaray does, is to translate the existing libaray, so it works with PSoC. THe project this is developed from is using CY8C888LTI-LP0097. There may be some other variation has different I2C APIs. if you are using an version that the I2C API is different, I hope this libaray will make your project easier by providing a template. 
