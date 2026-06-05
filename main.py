# MicroPython
# mail: goctaprog@gmail.com
# MIT license
# Пожалуйста, прочитайте документацию на QMC5883L!
# Please read the QMC5883L documentation!
import math
import time
from machine import I2C, Pin
from micropython import const

from qmc5883mod import QMC5883L
from sensor_pack_2.geosensmod import HardIronCalibrator, MagRange
from sensor_pack_2.bus_service import I2cAdapter

I2C_ID = const(1)
SCL_PIN = const(7)
SDA_PIN = const(6)
I2C_FREQ = const(400_000)
SENSOR_ADDR = const(0x0D)
ITERATIONS = const(33)

calibration_on: bool = True


def run_calibration(sens: QMC5883L, duration_ms=15_000) -> HardIronCalibrator:
    """Проводит процедуру калибровки."""
    width = 60
    print("\n" + "=" * width)
    print(" КАЛИБРОВКА ДАТЧИКА (Hard Iron Compensation)")
    print("=" * width)
    print("ИНСТРУКЦИЯ ДЛЯ ПОЛЬЗОВАТЕЛЯ:")
    print("Убедитесь, что рядом нет посторонних магнитов или металла.")
    print("Медленно вращайте датчик во всех направлениях (восьмерка/сфера).")
    print(f"Продолжайте вращение в течение {0.001 * duration_ms} секунд.")
    print("=" * width)

    # ВРЕМЯ ПОДГОТОВИТЬСЯ И ВЗЯТЬ ПЛАТУ С ДАТЧИКОМ В РУКИ!
    print("\nПодготовьтесь... Начало сбора данных через 3 секунды.")
    time.sleep(3)

    print("Сбор данных запущен! Начинайте вращать датчик...")

    cal = HardIronCalibrator()
    start_time = time.ticks_ms()
    samples = 0

    while time.ticks_diff(time.ticks_ms(), start_time) < duration_ms:
        if sens.is_data_ready():
            data = next(sens)
            if data is not None:
                cal.update(data)
                samples += 1
                if samples % 15 == 0:
                    print(".", end="")
        time.sleep_ms(10)

    print(f"\nСбор данных завершен. Собрано образцов: {samples}")

    if samples < 100:
        print("ВНИМАНИЕ: Собрано слишком мало данных. Калибровка может быть неточной.")

    cal.calculate_offsets()
    return cal

def show_calibration_offsets(cal: HardIronCalibrator):
    """Выводит вычисленные смещения (offsets) калибратора в консоль."""
    width = 45
    print("\n" + "=" * width)
    print(" РЕЗУЛЬТАТЫ КАЛИБРОВКИ (Смещения)")
    print("=" * width)
    if cal.is_calibrated():
        print(f" Смещение по оси X (Offset X): {cal.offset_x:>8.4f} G")
        print(f" Смещение по оси Y (Offset Y): {cal.offset_y:>8.4f} G")
        print(f" Смещение по оси Z (Offset Z): {cal.offset_z:>8.4f} G")
    else:
        print(" Калибровка не выполнена. Смещения равны 0.0000 G")
    print("=" * width + "\n")

def show_mode(sen: QMC5883L):
    """
    Выводит текущие настройки и статус датчика QMC5883L в консоль.
    Идеально подходит для отладки и проверки состояния оборудования.

    :param sen: Экземпляр класса QMC5883L
    """
    width = 40
    print("=" * width)
    print("       QMC5883L Sensor Status       ")
    print("=" * width)

    # Идентификация чипа
    chip_id = sen.get_id()
    id_status = "OK" if chip_id == 0xFF else f"ERROR (0x{chip_id:02X})"
    print(f"Chip ID         : 0x{chip_id:02X} ({id_status})")

    # Режим измерений
    mode_str = "Continuous (Непрерывный)" if sen.is_continuously_mode() else "Standby (Ожидание)"
    print(f"Measurement Mode: {mode_str}")

    # Формат возвращаемых данных
    format_str = "Raw (LSB)" if sen.is_raw_mode() else "Gauss (Гауссы)"
    print(f"Data Format     : {format_str}")

    # Частота обновления (ODR)
    odr_map = {0: "10 Hz", 1: "50 Hz", 2: "100 Hz", 3: "200 Hz"}
    odr_val = odr_map.get(sen.get_update_rate(), "Unknown")
    print(f"Update Rate     : {odr_val}")

    # Диапазон измерений (Full Scale)
    fs_str = "8 Gauss" if sen.is_full_scale() else "2 Gauss"
    print(f"Full Scale      : {fs_str}")

    # Передискретизация (Oversample Ratio)
    osr_map = {0: "512", 1: "256", 2: "128", 3: "64"}
    osr_val = osr_map.get(sen.get_oversample_rate(), "Unknown")
    print(f"Oversample (OSR): {osr_val}")

    print("-" * width)

    # Текущие флаги статуса
    try:
        status = sen.get_data_status(raw=False)
        drdy_str = "Ready" if status.DRDY else "Waiting"
        ovl_str = "YES (Error)" if status.OVL else "No"
        # Если DOR в Истина, это значит: ты пропустил одно или несколько измерений, старые данные потеряны (Lost)
        # Датчик настроен на непрерывные измерения с высокой частотой (например, 100 Гц или 200 Гц).
        # Микроконтроллер (ваша плата) не успел прочитать предыдущее измерение из регистров датчика.
        # Датчик сделал новое измерение и перезаписал старые данные в своих регистрах.
        dor_str = "YES (Lost)" if status.DataNotRead else "No"
        print(f"Status Flags    : DRDY: {drdy_str:<7} | OVL: {ovl_str:<9} | DOR: {dor_str}")
    except Exception as e:
        print(f"Status Flags    : Error reading status ({e})")

    # Температура
    try:
        temp = sen.get_temperature()
        print(f"Temperature     : {temp:.2f} C (относительная)")
    except Exception as e:
        print(f"Temperature     : Error reading ({e})")

    print("=" * width)


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    i2c = I2C(id=I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=I2C_FREQ)
    adapter = I2cAdapter(i2c)  # адаптер для стандартного доступа к шине
    delay_func = time.sleep_ms
    
    sensor = QMC5883L(adapter)
    print(f"Sensor id: {sensor.get_id()}")
    print(16 * "_")
    show_mode(sensor)

    # =====================================================================
    # ИНТЕРАКТИВНЫЙ ЗАПРОС НА КАЛИБРОВКУ.
    # Настраиваю датчик в стабильный режим для сбора данных калибровки.
    # =====================================================================
    sensor.set_magnitude_range_index(MagRange.G2)  # Для калибровки лучше использовать 2 Гаусса (выше разрешение)
    sensor.set_update_rate(2)     # 100 Hz
    sensor.set_oversample_rate(1) # OSR 256
    sensor.set_continuous_mode(True)
    sensor.set_raw_mode(False)    # данные сразу в Гауссах
    sensor.start_measurement()

    if not calibration_on:
        print("Калибровка пропущена. Используются нулевые смещения.")
        clbr = HardIronCalibrator()
    else:
        clbr = run_calibration(sensor)
        show_calibration_offsets(clbr)
    # =====================================================================

    # --- БЛОК 1: Измерения в диапазоне 2 Гаусса ---
    sensor.set_magnitude_range_index(MagRange.G2)  # Диапазон 2 Гаусса
    sensor.set_update_rate(2)     # ODR 100 Hz
    sensor.set_oversample_rate(1) # OSR 256
    sensor.set_continuous_mode(True)
    sensor.start_measurement()
    
    show_mode(sensor)
    wt = sensor.get_conversion_cycle_time()
    delay_func(wt)

    current_temp = sensor.get_temperature()
    print("\n--- Измерения: 2 Gauss Range ---")
    index = 0
    for mf_comp in sensor:
        delay_func(wt)
        if mf_comp:
            cal_data = clbr.apply(mf_comp)
            magnitude = math.sqrt(cal_data.x * cal_data.x + cal_data.y * cal_data.y + cal_data.z * cal_data.z)
            print(
                f"X: {cal_data.x:.4f}; Y: {cal_data.y:.4f}; Z: {cal_data.z:.4f}; temp.[C]: {current_temp:.2f}, {magnitude:.4f} [G]")
        index += 1
        if index > ITERATIONS:
            break

    # --- БЛОК 2: Измерения в диапазоне 8 Гаусс ---
    sensor.set_update_rate(index=3)     # 200 Hz
    sensor.set_magnitude_range_index(MagRange.G8) # 8 Гаусс
    sensor.set_oversample_rate(index=3) # OSR 64
    sensor.set_continuous_mode(continuous=True)
    sensor.start_measurement()
    
    wt = sensor.get_conversion_cycle_time()
    show_mode(sensor)
    delay_func(wt)

    current_temp = sensor.get_temperature()
    print("\n--- Измерения: 8 Gauss Range ---")
    index = 0
    for mf_comp in sensor:
        delay_func(wt)
        if mf_comp:
            cal_data = clbr.apply(mf_comp)
            magnitude = math.sqrt(cal_data.x * cal_data.x + cal_data.y * cal_data.y + cal_data.z * cal_data.z)
            print(
                f"X: {cal_data.x:.4f}; Y: {cal_data.y:.4f}; Z: {cal_data.z:.4f}; temp.[C]: {current_temp:.2f}, {magnitude:.4f} [G]")
        index += 1
        if index > ITERATIONS:
            break