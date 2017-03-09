sample.c is the device driver, a loadable kernel module and implements a character based device, in particular the driver for the HCSR04 sensor. 
app.c is the test program that allows you to use, from user space, the services provided by the device driver through the Virtual File Interface.
for more details, see pdf file.