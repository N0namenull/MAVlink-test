from uav_controller import UAVController

if __name__ == "__main__":

    uav = UAVController('udp:127.0.0.1:14550')

    try:
        import time
        time.sleep(2)

        print("Добро пожаловать в консоль управления БПЛА.")
        print("Доступные команды:")
        print("  takeoff <высота>          - Взлет на заданную высоту")
        print("  land                      - Посадка")
        print("  rtl                       - Возврат к точке взлета")
        print("  reposition <широта> <долгота> <высота> - Перемещение к новым координатам")
        print("  get_params                - Вывод параметров БПЛА")
        print("  exit                      - Выход из программы")

        while True:
            command = input("Введите команду: ").strip()

            if command.startswith("takeoff"):
                parts = command.split()
                if len(parts) == 2:
                    altitude = float(parts[1])
                    uav.takeoff(altitude)
                else:
                    print("Неверный формат команды. Используйте: takeoff <высота>")

            elif command == "land":
                uav.land()

            elif command == "rtl":
                uav.return_to_launch()

            elif command.startswith("reposition"):
                parts = command.split()
                if len(parts) == 4:
                    latitude = float(parts[1])
                    longitude = float(parts[2])
                    altitude = float(parts[3])
                    uav.reposition(latitude, longitude, altitude)
                else:
                    print("Неверный формат команды. Используйте: reposition <широта> <долгота> <высота>")

            elif command == "get_params":
                params = uav.get_parameters()
                for msg_type, data in params.items():
                    print(f"{msg_type}: {data}")

            elif command == "exit":
                print("Завершение программы...")
                break

            else:
                print("Неизвестная команда. Введите 'help' для списка доступных команд.")

    except KeyboardInterrupt:
        print("Прерывание программы пользователем.")

    finally:
        uav.stop()
