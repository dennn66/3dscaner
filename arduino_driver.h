
/**
 *  Get Arduino device file name from command line arguments.
 */
char* getDeviceFileName(int argc, char **argv);

/**
 *  Open Arduino device file.
 */
int openDeviceFile(const char* fileName, const char* mode);


/**
 *  Get baud rate from command line arguments.
 */
unsigned int getBaudRate(int argc, char **argv);

/**
 *  Set Arduino serial port attributes.
 */
void setAttr(int fd, unsigned int baud);

/**
 *  Send command to Arduino.
 */
void sendCommand(int fd, const char *cmd);


/**
 *  Display command result from Arduino.
 */
void displayResult(int fd);


