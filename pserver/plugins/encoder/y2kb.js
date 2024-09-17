const i2c = require('i2c-bus');

// Constants for I2C communication
const I2C_SLAVE_DEV_ADDR = 0x11;

// Register addresses
const CVAL_REG_ADDR = 0x03;
const ECONF_REG_ADDR = 0x0B;

const busNumber = 1; // Check your I2C bus number on Jetson, typically 1
const i2cBus = i2c.openSync(busNumber);

// Helper function to write a value to a specific register
function writeRegister(register, value) {
  i2cBus.writeByteSync(I2C_SLAVE_DEV_ADDR, register, value);
}

// Helper function to read 4 bytes from a register
function readRegister(register, length) {
  const buffer = Buffer.alloc(length);
  i2cBus.readI2cBlockSync(I2C_SLAVE_DEV_ADDR, register, length, buffer);
  return buffer;
}

// Main function to configure and read the encoder
async function main() {
  try {
    // Set encoder config (ECONF: Data type=int32, Count dir=Up, Multiply=x2_A, Phase=A,B)
    const econf = 0x07;
    writeRegister(ECONF_REG_ADDR, econf);
    
    // Wait for 4 seconds to let the device reboot after setting config
    await new Promise(resolve => setTimeout(resolve, 4000));

    // Periodically read the encoder count
    setInterval(() => {
      // Read 4 bytes from the encoder count register
      const countBytes = readRegister(CVAL_REG_ADDR, 4);
      const count = countBytes.readInt32LE(0); // Convert to signed 32-bit integer
      
      // Output the encoder count
      console.log(`Count: ${count}`);
    }, 100); // Read every 100 ms

  } catch (error) {
    console.error("Error:", error);
  }
}

main();
