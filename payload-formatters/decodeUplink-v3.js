function decodeUplink(input) {
  let data = {};
  let errors = [];

  try {
      // Validate fPort
      if (input.fPort !== 10) {
          throw new Error("Invalid fPort: " + input.fPort);
      }

      // Validate payload length (should be at least 12 bytes)
      if (!input.bytes || input.bytes.length < 12) {
          throw new Error("Unexpected payload length: " + (input.bytes ? input.bytes.length : 0));
      }

      // Decode Latitude
      data.latitude = ((input.bytes[0] << 16) >>> 0) + ((input.bytes[1] << 8) >>> 0) + input.bytes[2];
      data.latitude = (data.latitude / 16777215.0 * 180) - 90;

      // Decode Longitude
      data.longitude = ((input.bytes[3] << 16) >>> 0) + ((input.bytes[4] << 8) >>> 0) + input.bytes[5];
      data.longitude = (data.longitude / 16777215.0 * 360) - 180;

      // Decode Altitude (handling negative values)
      let altValue = ((input.bytes[6] << 8) >>> 0) + input.bytes[7];
      let sign = input.bytes[6] & (1 << 7);
      data.altitude = sign ? (0xFFFF0000 | altValue) : altValue;

      // Decode HDOP (Horizontal Dilution of Precision)
      data.hdop = input.bytes[8] / 10;

      // Decode Number of Satellites
      data.sats = input.bytes[9];

      // Decode Battery Voltage (vBat) - 2 bytes
      let vbatRaw = (input.bytes[10] << 8) | input.bytes[11];  // Combine HIGH and LOW bytes
      data.vBat = (vbatRaw + 200) * 10;  // Convert back to millivolts (mV)

  } catch (err) {
      errors.push("Decoding error: " + err.message);
      return { errors: errors };
  }

  return {
      data: data,
      errors: errors.length > 0 ? errors : undefined
  };
}
