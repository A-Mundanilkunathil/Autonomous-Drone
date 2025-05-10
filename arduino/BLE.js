const noble = require('@abandonware/noble');

const SERVICE_UUID = '180c';
const CHARACTERISTIC_UUID = '2a56';
const SPEED = 1000; // Change this to your desired value

noble.on('stateChange', async (state) => {
  if (state === 'poweredOn') {
    console.log('Scanning...');
    noble.startScanning([SERVICE_UUID], false);
  }
});

noble.on('discover', async (peripheral) => {
  console.log('Found device:', peripheral.advertisement.localName);

  if (peripheral.advertisement.localName === 'NanoBLE-ESC') {
    noble.stopScanning();
    await peripheral.connectAsync();
    console.log('Connected');

    const { characteristics } = await peripheral.discoverSomeServicesAndCharacteristicsAsync(
      [SERVICE_UUID],
      [CHARACTERISTIC_UUID]
    );

    const buffer = Buffer.alloc(4);
    buffer.writeInt32LE(SPEED);
    await characteristics[0].writeAsync(buffer, false);
    console.log('Speed sent:', SPEED);

    await peripheral.disconnectAsync();
    console.log('Disconnected');
    process.exit(0);
  }
});
