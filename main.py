from network.udp_manager import UdpManager
from autonomous_driving.autonomous_driving import AutonomousDriving
from safety_shield.safety_shield import SafetyShield


def main():
    autonomous_driving = AutonomousDriving()
    safety_shield = SafetyShield()
    udp_manager = UdpManager(autonomous_driving, safety_shield)
    udp_manager.execute()


if __name__ == '__main__':
    main()
