# Protocols

UART (Universal Asynchronous Receiver/Transmitter) is a widely used asynchronous serial communication protocol that transmits and receives data without a shared clock, relying instead on start and stop bits to frame the data stream. It is simple, supports point-to-point communication, and is ideal for low-pin-count, short-range data links between devices such as microcontrollers and PCs. 
I2C (Inter-Integrated Circuit) is a synchronous, two-wire communication protocol using a shared clock (SCL) and data line (SDA). It supports multiple masters and multiple slaves on the same bus, with each slave device having a unique address, making it very efficient for connecting several low-speed peripherals like sensors, EEPROMs, or displays to a single controller. 
Together, these protocols form the backbone of many embedded system interfaces by providing reliable and flexible methods to transfer serial data.
