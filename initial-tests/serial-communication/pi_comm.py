import serial

def main():
    # Open serial port
    # /dev/serial/by-id/usb-Raspberry_Pi_Pico_E6625887D3859130-if00 - Pranav
    # /dev/serial/by-id/usb-Raspberry_Pi_Pico_E6625887D3482132-if00 - Adbhut
    ser = serial.Serial('/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6625887D3859130-if00', 115200, timeout=1)
    print("Enter speed and steering:")
    while True:
        try:
            #global user_input, prev_user_input
            # User input

            user_input = input("Speed Steering > ")            

            parts = user_input.split()
            if len(parts) != 2:
                print("Please enter speed and steering")
                continue
            speed = float(parts[0])
            steering = float(parts[1])

            # Send command to RP2040
            command = f"{speed},{steering}\n"
            ser.write(command.encode())

            # Wait for response from RP2040
            response = ser.readline().decode().strip()
            print(response)
            values = response.split(",")
            yaw = float(values[0])
            distance = -float(values[14]) / 30
            left_dist = int(values[12])
            front_dist = int(values[9])
            right_dist = int(values[10])
            print(f"Received Data - Yaw: {yaw}, Distance: {distance} \n\tLeft: {left_dist}, Front: {front_dist}, Right: {right_dist}")


        except KeyboardInterrupt:
            print("\nExiting.")
            break
        except Exception as expt:
            print(f"Error: {expt}")

if __name__ == "__main__":
    main()
