import serial
import time

def send_packet(ser, usbl_stat):
    # Dummy variables
    lat = 37.7749       # latitude (float)
    long = -122.4194    # longitude (float)
    dvl_stat = 1        # DVL status (int)
    gps_stat = 1        # GPS status (int)
    pa_stat = 1         # PA status (int)
    roll = 10           # roll (int)
    pitch = 5           # pitch (int)
    yaw = 90            # yaw (int)

    # Create the data packet
    packet = f"L{lat}O{long}D{dvl_stat}U{usbl_stat}G{gps_stat}P{pa_stat}R{roll}W{pitch}Y{yaw}E*"
    
    # Send the packet
    ser.write(packet.encode('utf-8'))
    
    print(f"Sent packet: {packet}")

def receive_packet(ser):
    # Read the packet
    packet = ser.read_until(b'E').decode('utf-8')
    
    print(f"Received packet: {packet}")

    # Parse the packet
    try:
        mode = int(packet[packet.index('A') + 1:packet.index('L')])
        rec_lat = float(packet[packet.index('L') + 1:packet.index('O')])
        rec_long = float(packet[packet.index('O') + 1:packet.index('S')])
        sweep_time = int(packet[packet.index('S') + 1:packet.index('F')])
        set_freq = int(packet[packet.index('F') + 1:packet.index('E')])

        return {
            "mode": mode,
            "lat": rec_lat,
            "long": rec_long,
            "sweep_time": sweep_time,
            "set_freq": set_freq
        }
    except ValueError as e:
        print(f"Error parsing packet: {e}")
        return None

def main():
    port = 'COM11'  
    baudrate = 115200  

    ser = serial.Serial(port, baudrate, timeout=1)
    
    usbl_stat = 0
    
    try:
        start_time = time.time()
        while True:
            send_packet(ser, usbl_stat)
            time.sleep(0.1)  
            received_data = receive_packet(ser)
            if received_data:
                print(f"Parsed data: {received_data}")
            if time.time() - start_time >= 2:
                usbl_stat = 1 - usbl_stat  
                start_time = time.time()  
            time.sleep(0.5)  
    except KeyboardInterrupt:
        print("Process interrupted by user")
    finally:
        ser.close()
        print("Serial connection closed")

if __name__ == "__main__":
    main()
