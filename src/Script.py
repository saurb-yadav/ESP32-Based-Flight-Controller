import pygame
import socket
import time

# UDP configuration
UDP_IP = "192.168.4.1"  # ESP32 SoftAP default IP
UDP_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Initialize Pygame and joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect an Xbox controller.")
    exit(1)

controller = pygame.joystick.Joystick(0)
controller.init()
print(f"Controller connected: {controller.get_name()}")

# Control variables
throttle = 0  # 0 to 1000 (maps to 1000-2000 µs on ESP32)
pitch = 0.0   # -30 to 30 degrees
roll = 0.0    # -30 to 30 degrees
yaw = 0.0     # -180 to 180 degrees
armed = False
last_throttle_change = 0  # Debounce for throttle buttons

def send_udp_message(message):
    """Send a UDP message to the ESP32."""
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
    print(f"Sent: {message}")

def map_joystick_axis(value, deadzone=0.1, max_output=30.0):
    """Map joystick axis (-1 to 1) to output range with deadzone."""
    if abs(value) < deadzone:
        return 0.0
    # Scale to max_output, preserving sign
    return max_output * value

def main():
    global throttle, armed, last_throttle_change

    running = True
    clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Button press events
            if event.type == pygame.JOYBUTTONDOWN:
                current_time = pygame.time.get_ticks()

                # Y button: Increase throttle by 100
                if event.button == 3:  # Y button
                    if current_time - last_throttle_change > 200:  # Debounce 200ms
                        throttle = min(throttle + 100, 1000)
                        last_throttle_change = current_time
                        print(f"Throttle increased to: {throttle}")

                # A button: Decrease throttle by 100
                if event.button == 0:  # A button
                    if current_time - last_throttle_change > 200:  # Debounce 200ms
                        throttle = max(throttle - 100, 0)
                        last_throttle_change = current_time
                        print(f"Throttle decreased to: {throttle}")

                # Menu button: ARM
                if event.button == 7:  # Menu button
                    if not armed:
                        send_udp_message("ARM")
                        armed = True

                # View button: DISARM
                if event.button == 6:  # View button
                    if armed:
                        send_udp_message("DISARM")
                        armed = False
                        throttle = 0  # Reset throttle on disarm

                # B button: Reset yaw
                if event.button == 1:  # B button
                    send_udp_message("RESET_YAW:1")
                    print("Yaw reset")

        # Read joystick axes
        # Left joystick: X-axis (roll), Y-axis (pitch)
        roll = map_joystick_axis(controller.get_axis(0), deadzone=0.1, max_output=30.0)
        pitch = map_joystick_axis(-controller.get_axis(1), deadzone=0.1, max_output=30.0)  # Invert Y-axis for intuitive pitch

        # Right joystick: X-axis (yaw), Y-axis (unused)
        yaw = map_joystick_axis(controller.get_axis(3), deadzone=0.1, max_output=180.0)
        yaw = 0
        # Send control message if armed
        if armed:
            control_message = f"({throttle},{pitch:.2f},{roll:.2f},{yaw:.2f})"
            send_udp_message(control_message)

        # Maintain 50 Hz update rate
        clock.tick(50)

    # Cleanup
    pygame.quit()
    sock.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program terminated.")
        pygame.quit()
        sock.close()
