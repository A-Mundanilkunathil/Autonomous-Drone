from sense_hat import SenseHat

sense = SenseHat()

def get_sensor_data():
    # Get tempearture
    temperature = sense.get_temperature()
    
    # Get humidity
    humidity = sense.get_humidity()
    
    # Get pressure
    pressure = sense.get_pressure()
    
    # Get gyroscope data
    gyroscope = sense.get_gyroscope()
    
    # Get accelerometer data
    accelerometer = sense.get_accelerometer()
    
    # Get magnetometer data
    magnetometer = sense.get_compass_raw()
    
    # Get orientation
    orientation = sense.get_orientation()
    
    # Get light level
    light = sense.get_light()

# Example usage
if __name__ == "__main__":
    data = get_sensor_data()
    print("Temperature: ", data['temperature'])
    print("Humidity: ", data['humidity'])
    print("Pressure: ", data['pressure'])
    print("Gyroscope: ", data['gyroscope'])
    print("Accelerometer: ", data['accelerometer'])
    print("Magnetometer: ", data['magnetometer'])
    print("Orientation: ", data['orientation'])
    print("Light Level: ", data['light'])