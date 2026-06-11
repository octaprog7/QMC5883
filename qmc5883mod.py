"""MicroPython module for QMC5883L or HMC5883L Geomagnetic Sensor"""
from micropython import const
from collections import namedtuple

from sensor_pack_2.geosensmod import AXIS_ALL, MagRange
from sensor_pack_2.bus_service import I2cAdapter
from sensor_pack_2.geosensmod import (MagnetometerData, UpdateRates, OversampleLevels, PerformanceProfile,
                                      ICommonMagnitometer, axis_index_to_reg_addr, check_axis_index, PerformanceProfiles)
from sensor_pack_2.base_sensor import IDentifier, DeviceEx, check_value

# КОРТЕЖ КОРТЕЖЕЙ!
# Карта профилей производительности для QMC5883L.
# Индексы (0-4) строго соответствуют значениям в классе PerformanceProfiles.
_PROFILE_MAP = (
    # 0: HIGH_ACCURACY/ВЫСОКАЯ_ТОЧНОСТЬ (10 Гц + Макс. усреднение)
    # ДЛЯ ЧЕГО: Стационарный компас, геодезия, устройство лежит на столе.
    # ДАТЧИК работает медленно, но очень тщательно фильтрует шум.
    # Показания максимально стабильные, стрелка компаса не "дрожит".
    PerformanceProfile(UpdateRates.HZ_10, OversampleLevels.ULTRA_HIGH),
    # 1: BACKGROUND_MONITORING/ФОНОВЫЙ_МОНИТОРИНГ (10 Гц + Мин. усреднение)
    # ДЛЯ ЧЕГО: Устройства на батарейке, которые должны работать месяцами.
    # ДАТЧИК просыпается редко и не тратит энергию на сложные вычисления.
    # Точность ниже, но для простого определения "север-юг" этого достаточно.
    PerformanceProfile(UpdateRates.HZ_10, OversampleLevels.MEDIUM_LOW),
    # 2: DYNAMIC_NAVIGATION/ДИНАМИЧЕСКАЯ_НАВИГАЦИЯ (50 Гц + Баланс)
    # ДЛЯ ЧЕГО: Роботы, машинки на радиоуправлении, пешая навигация.
    # ДАТЧИК находит золотую середину: успевает реагировать на повороты,
    # но при этом сигнал остается достаточно чистым и без сильных скачков.
    PerformanceProfile(UpdateRates.HZ_50, OversampleLevels.BALANCED),
    # 3: TILT_COMPENSATION (100 Гц + Баланс)
    # ДЛЯ ЧЕГО: Работа в паре с акселерометром или гироскопом (фильтры Калмана, Маджвика).
    # ДАТЧИК выдает данные с идеальной для математических фильтров частотой.
    # Это стандарт индустрии для расчета 3D-азимута с компенсацией наклона.
    PerformanceProfile(UpdateRates.HZ_100, OversampleLevels.BALANCED),

    # 4: БЫСТРЫЙ_ОТКЛИК (200 Гц + Макс. скорость)
    # ДЛЯ ЧЕГО: Простые следящие механизмы или детектирование резких событий.
    # ВНИМАНИЕ: На этой частоте MicroPython может не успевать выполнять сложную
    # математику (например, фильтры Калмана) в реальном времени. Используйте
    # этот режим для сбора "сырых" данных или простых пороговых реакций.
    PerformanceProfile(UpdateRates.HZ_200, OversampleLevels.HIGH_SPEED)
)


# КОНСТАНТЫ ПЕРЕВОДА (LSB -> Гаусс)
# Вычислены как: 1.0 / Чувствительность (из Таблицы 2 даташита)
_MULTIPLIER_2G = const(8.333333e-05)  # ~1.0 / 12000.0
_MULTIPLIER_8G = const(3.333333333333333e-04)   # 1.0 / 3000.0
# 1 / ODR [ms]
_dly_ms = const((100, 20, 10, 5))
#
_ADDR_STATUS_FLAGS_REG = const(0x06)
_ADDR_TEMP_REG = const(0x07) # два байта
_ADDR_CTRL_1_REG = const(0x09)
_ADDR_CTRL_2_REG = const(0x0A)
_ADDR_PERIOD_REG = const(0x0B)
_ADDR_ID_REG = const(0x0D)
# DRDY: Data ready 1 или нет 0.
# DataNotRead (DOR): Если фрагмент данных не был прочитан, текущие данные будут сброшены при поступлении следующих данных.
# В этом случае прерывание (бит DRDY) остается в высоком состоянии до тех пор, пока не будут считаны данные.
# Бит DOR устанавливается в «1», что указывает на потерю набора данных измерений.
# Бит DOR переходит в «0» при следующем обращении к регистру 0x06 (_ADDR_STATUS_FLAGS_REG) в операции чтения данных.
# OVL: 0 - нет переполнения, 1 - переполнение данных.
# Флаг переполнения (OVL) устанавливается в 1, если какие-либо данные трех каналов магнитного датчика выходят за пределы допустимого диапазона.
# Выходные данные каждой оси насыщаются в диапазонах -32768 и 32767; если какая-либо из осей выходит за эти пределы,
# флаг OVL устанавливается в 1. Этот флаг сбрасывается в значение 0, если следующее измерение возвращается в диапазон (-32768, 32767), в противном случае он остается равным 1.
DataStatus = namedtuple("DataStatus", "DataNotRead OVL DRDY")


def _raw_int_to_gauss(raw_val: int, is_8g: bool = True) -> float:
    """
    Преобразует сырые знаковые целочисленные значения осей в Гауссы.

    :param raw_val: Знаковое сырое значение оси X (-32768..32767)
    :param is_8g: True, если датчик настроен на диапазон +/-8G (RNG=01),
                  иначе False (диапазон +/-2G, RNG=00)
    :return: float, значения в Гауссах
    """
    mult = _MULTIPLIER_8G if is_8g else _MULTIPLIER_2G
    return raw_val * mult

def _get_val(raw_val: int, is_8g: bool, raw_out: bool = True) -> int | float:
    """Возвращает значение в зависимости от входных параметров.
    raw_val - сырое значение, полученное от датчика;
    is_8g - True, если датчик настроен на диапазон +/-8G (RNG=01);
    raw_out - если истина, то возвращает сырое значение, иначе возвращает значение в Гауссах;
    """
    if raw_out:
        return raw_val
    return _raw_int_to_gauss(raw_val, is_8g)

class QMC5883L(ICommonMagnitometer, IDentifier):
    """QMC5883L or HMC5883L Geomagnetic Sensor."""

    def __init__(self, adapter: I2cAdapter, address: int = 0x0D):
        check_value(value=address, valid_range=(0x0D,), error_msg=f"Invalid address value: {address}")
        self._connection = DeviceEx(adapter=adapter, address=address, big_byte_order=False)
        self._buf_2 = bytearray(2)  # для хранения
        self._buf_6 = bytearray(6)  # для хранения
        #
        self._update_rate_index = UpdateRates.HZ_10
        self._continuous_mode = False
        # диапазон напряженности магнитного поля на который1 настроен датчик
        self._magnitude_range_index = MagRange.G2
        self._over_sample_index = OversampleLevels.ULTRA_HIGH
        self._performance_profile = PerformanceProfiles.HIGH_ACCURACY  # Дефолтный профиль
        # если Истина, то get_measurement_value возвращает результат в безразмерных (сырых значениях)
        # если Ложь, то get_measurement_value возвращает результат в Гауссах!
        self._raw_mode = False
        #
        self.setup()
        self.refresh_config()

    def set_raw_mode(self, value: bool | None = None) -> bool:
        """Устанавливает тип значения, возвращаемого методом get_measurement_value.
        Если value Истина, то get_measurement_value возвращает сырые безразмерные значения.
        Если value Ложь, то get_measurement_value возвращает значения в Гаусс.
        Значение используется методом start_measurement!
        Возвращает текущее значение типа значения, возвращаемого методом get_measurement_value.
        """
        if value is None:
            return self._raw_mode
        self._raw_mode = value
        return value

    def set_update_rate_index(self, index: int | None = None) -> int:
        """
        Устанавливает или возвращает частоту обновления данных (ODR).

        :param index: Значение из UpdateRates (HZ_10, HZ_50, HZ_100, HZ_200)
                      или целое число (0, 1, 2, 3). Если None, возвращает текущее значение.
        :return: Текущее значение частоты (int).
        """
        if index is None:
            return self._update_rate_index

        # поддерживаемые частоты для QMC5883L.
        # работает и с UpdateRates.HZ_10, и с int числом 0
        allowed_rates = (
            UpdateRates.HZ_10,
            UpdateRates.HZ_50,
            UpdateRates.HZ_100,
            UpdateRates.HZ_200
        )

        if index not in allowed_rates:
            raise ValueError(f"Поддерживает только частоты 10, 50, 100 или 200 Гц. Получено недопустимое значение: {index}.")

        self._update_rate_index = index
        return index

    def set_continuous_mode(self, value: bool | None = None) -> bool:
        """Устанавливает режим измерений.
        Значение используется методом start_measurement."""
        if value is None:
            return self._continuous_mode
        self._continuous_mode = value
        return value

    def set_magnitude_range_index(self, range_idx: int | None = None) -> int:
        """Устанавливает диапазон измерения напряженности магнитного поля по индексу.
        :param range_idx: MagRange.G2 (0) или MagRange.G8 (1)"""
        if range_idx is None:
            return self._magnitude_range_index
        if range_idx != MagRange.G2 and range_idx != MagRange.G8:
            raise ValueError(f"QMC5883L не поддерживает индекс {range_idx}!")
        self._magnitude_range_index = range_idx
        return range_idx

    def set_oversample_index(self, index: int | None = None) -> int:
        """Устанавливает или возвращает индекс уровня передискретизации (OSR).
        QMC5883L поддерживает только 4 аппаратных уровня (0, 1, 2, 3)."""
        if index is None:
            return self._over_sample_index
        # Масштабирование: Если передан индекс 4 или 5 (например, HIGH_SPEED),
        # функция min() ограничит его значением 3 (максимум для QMC).
        # Если передан в диапазоне от 0 до 3, он останется без изменений.
        osi = max(0, min(index, 3))
        self._over_sample_index = osi
        return osi

    def set_performance_profile(self, profile: int | PerformanceProfile | None = None) -> PerformanceProfile:
        if profile is None:
            return PerformanceProfile(
                update_rate=self.set_update_rate_index(),
                oversample=self.set_oversample_index()
            )

        if isinstance(profile, int):
            if not (0 <= profile < len(_PROFILE_MAP)):
                raise ValueError(f"Неизвестный профиль производительности: {profile}")
            target_profile = _PROFILE_MAP[profile]
            # Сохраняю индекс профиля
            self._performance_profile = profile

        elif isinstance(profile, PerformanceProfile):
            target_profile = profile
            # Сохраняю кортеж профиля
            self._performance_profile = profile
        else:
            raise TypeError("profile должен быть int или PerformanceProfile")

        self.set_update_rate_index(target_profile.update_rate)
        self.set_oversample_index(target_profile.oversample)

        return PerformanceProfile(
            update_rate=self._update_rate_index,
            oversample=self._over_sample_index
        )

    def _get_ctrl_1(self) -> int:
        """возвращает содержимое первого(!) регистра управления"""
        conn = self._connection
        ctrl_1 = conn.read_reg(reg_addr=_ADDR_CTRL_1_REG, bytes_count=1)[0]
        return ctrl_1

    # IDentifier
    def get_id(self):
        """Возвращает значение (Chip ID), которое равно 0xFF!"""
        return self._connection.read_reg(reg_addr=_ADDR_ID_REG, bytes_count=1)[0]

    def soft_reset(self):
        """Выполняет программный сброс датчика"""
        conn = self._connection
        conn.write_reg(reg_addr=_ADDR_CTRL_2_REG, value=0x80, bytes_count=1)

    def get_temperature(self) -> float:
        """Возвращает температуру, измеренную датчиком.
        offset - смещение в градусах Цельсия. Нужно подбирать по эталонному термометру!
        coefficient - коэфф. преобразования сырого значения в градусы Цельсия. Лучше не изменять!
        Из даташита: «Коэффициент усиления датчика температуры откалиброван на заводе, но его смещение не компенсировано,
        поэтому точным является только относительное значение температуры».
        """
        buf = self._buf_2
        conn = self._connection
        coefficient = 0.01
        conn.read_buf_from_mem(address=_ADDR_TEMP_REG, buf=buf)  # 16 bit value (int16)
        return coefficient * conn.unpack(fmt_char="h", source=buf)[0]  # h - signed short

    def is_single_shot_mode(self) -> bool:
        """Датчик не поддерживает этот режим!"""
        return False

    def is_continuously_mode(self):
        """Возвращает Истина, когда включен режим периодических измерений!"""
        return 0 != (0x01 & self._get_ctrl_1())

    def in_standby_mode(self) -> bool:
        """Возвращает Истина, когда включен режим ожидания(экономичный режим)!"""
        return 0 == (0x01 & self._get_ctrl_1())

    def get_data_status(self, raw: bool = False) -> int | DataStatus:
        """Возвращает кортеж битов(номер бита): Data Skip (DOR) (2), Overflow flag (OVL) (1), Data Ready (0)"""
        conn = self._connection
        stat = conn.read_reg(reg_addr=_ADDR_STATUS_FLAGS_REG, bytes_count=1)[0]
        if raw:
            return stat
        return DataStatus(DataNotRead=0 != (stat & 0x04), OVL=0 != (stat & 0x02), DRDY=0 != (stat & 0x01))

    def is_data_ready(self) -> bool:
        """Возвращает флаг Data Ready (DRDY)"""
        return self.get_data_status(raw=False).DRDY

    def start_measurement(self):
        """Запускает периодические измерения (continuous_mode is True) или переводит датчик в
        режим ожидания (continuous_mode is False).
        update_rate: 0-10 Hz; 1-50 Hz; 2-100 Hz; 3-200 Hz.  For most of compassing applications, recommend 10 Hz!
        full_scale: False-2 Gauss; True-8 Gauss;            Field ranges of the magnetic sensor!
        over_sample_ratio: 0-512; 1-256; 2-128; 3-64.       Larger OSR value leads to smaller filter bandwidth,
                                                            less in-band noise and higher power consumption."""
        osr = self._over_sample_index
        full_scale = MagRange.G8 == self.set_magnitude_range_index()
        check_value(self._update_rate_index, range(4), f"Invalid update rate: {self._update_rate_index}")
        check_value(osr, range(4), f"Invalid over sample ratio: {osr}")
        ctrl_reg1_val = (osr << 6) | (int(full_scale) << 4) | (self._update_rate_index << 2) | int(self._continuous_mode)

        conn = self._connection
        conn.write_reg(reg_addr=_ADDR_CTRL_1_REG, value=ctrl_reg1_val, bytes_count=1)

    def _get_single_axis_raw(self, axis_index: int) -> int:
        """Возвращает сырое значение магнитного поля по одной оси: (0-X, 1-Y, 2-Z)."""
        check_axis_index(axis_index)
        addr_reg = axis_index_to_reg_addr(axis_index, offset=0, multiplier=2)
        buf = self._buf_2
        conn = self._connection
        conn.read_buf_from_mem(address=addr_reg, buf=buf)     # 16 bit value (int16)
        return conn.unpack(fmt_char='h', source=buf)[0]

    def _get_all_axis_raw(self) -> tuple:
        """(x_raw, y_raw, z_raw)"""
        buf = self._buf_6
        conn = self._connection
        conn.read_buf_from_mem(address=0, buf=buf)
        return conn.unpack(fmt_char='hhh', source=buf)

    def get_measurement_value(self, value_index: int) -> None | int | float | MagnetometerData:
        """
        Возвращает измеренное значение магнитного поля по заданной оси или по всем осям.

        :param value_index: Индекс запрашиваемой оси.
                            AXIS_X - ось X, AXIS_Y - ось Y, AXIS_Z - ось Z, AXIS_ALL - все оси (X, Y, Z).
        :return:
            - Если value_index != AXIS_ALL: возвращает `int` (сырое значение) или `float` (в Гауссах).
            - Если value_index == AXIS_ALL: возвращает именованный кортеж `MagnetometerData` (x, y, z, is_raw).
            - Возвращает `None`, если произошла ошибка чтения или некорректный индекс.
            Для безопасного чтения рекомендуется:
            1. Предварительно вызывать `self.is_data_ready()`.
            2. Или использовать итератор `__next__()`, который выполняет проверку DRDY автоматически.

        Примечание:
            - Формат возвращаемых данных (сырые LSB или Гауссы) зависит от флага,
              установленного методом `set_raw_mode()`.
            - Коэффициент пересчета в Гауссы зависит от диапазона, заданного методом `set_full_scale()`.
        """
        raw_mode = self._raw_mode
        fsr = MagRange.G8 == self.set_magnitude_range_index()
        if AXIS_ALL != value_index:
            # запрос значения по одной оси
            raw_val = self._get_single_axis_raw(value_index)
        else:
            # запрос значения по ВСЕМ осям
            raw_val = self._get_all_axis_raw()
        #
        if isinstance(raw_val, int):
            return _get_val(raw_val, fsr, raw_mode)
        if isinstance(raw_val, tuple):
            return MagnetometerData(x=_get_val(raw_val[0], fsr, raw_mode), y=_get_val(raw_val[1], fsr, raw_mode),
                                    z=_get_val(raw_val[2], fsr, raw_mode), is_raw=raw_mode)
        return None

    def get_conversion_cycle_time(self) -> int:
        """Возвращает время, в миллисекундах, преобразования датчиком в зависимости от его настроек.
        Для режима периодических измерений, устанавливает частоту обновления значений величины магнитного поля
        update_rate должно быть в диапазоне от 0 до 3 включительно, что соответствует частотам:
        0 - 10 Hz; 1 - 50 Hz; 2 - 150 Hz; 2 - 100 Hz; 3 - 200 Hz"""
        upd_rate = self._update_rate_index
        check_value(upd_rate, range(4), f"Invalid update rate: {upd_rate}")
        return _dly_ms[upd_rate]

    def setup(self):
        """Запись значений по умолчанию в регистры датчика."""
        # roll-over function disabled, INT_ENB: “0”: enable interrupt PIN, “1”: disable interrupt PIN
        # bit_name      bit number
        # SOFT_RST      7
        # ROL_PNT       6
        # INT_ENB       0
        conn = self._connection
        conn.write_reg(reg_addr=_ADDR_CTRL_2_REG, value=0x00, bytes_count=1)
        # SET/RESET Period. It is recommended that the register 0BH is written by 0x01.
        conn.write_reg(reg_addr=_ADDR_PERIOD_REG, value=0x01, bytes_count=1)

    def refresh_config(self):
        """
        Читает текущую конфигурацию из регистра управления 1 (0x09)
        и обновляет внутренние поля экземпляра класса в соответствии с ними.
        Это гарантирует синхронизацию программного состояния с аппаратным.
        """
        conn = self._connection
        # 1 байт из регистра Control Register 1 (0x09)
        ctrl_1 = conn.read_reg(reg_addr=_ADDR_CTRL_1_REG, bytes_count=1)[0]

        # OSR (Over Sample Ratio) - биты 7 и 6
        # 00=512(0), 01=256(1), 10=128(2), 11=64(3)
        self._over_sample_index = (ctrl_1 >> 6) & 0x03

        # RNG (Full Scale Range) - биты 5 и 4
        # 00=2G, 01=8G. (10 и 11 зарезервированы)
        # Если значение равно 1 (0b01), значит включен диапазон 8G (True)
        rng_val = (ctrl_1 >> 4) & 0x03
        self.set_magnitude_range_index(MagRange.G8 if rng_val == 0b01 else MagRange.G2)

        # ODR (Output Data Rate) - биты 3 и 2
        # 00=10Hz(0), 01=50Hz(1), 10=100Hz(2), 11=200Hz(3)
        self._update_rate_index = (ctrl_1 >> 2) & 0x03

        # MODE - биты 1 и 0
        # 00=Standby, 01=Continuous. (10 и 11 зарезервированы)
        # Если значение равно 1 (0b01), значит включен непрерывный режим (True)
        mode_val = ctrl_1 & 0x03
        self._continuous_mode = (mode_val == 1)

    def __next__(self) -> None | MagnetometerData:
        """возвращает результат только в режиме периодических измерений!"""
        if self.is_continuously_mode() and self.is_data_ready():
            return self.get_measurement_value(AXIS_ALL)
        return None

    def get_adc_conversion_time(self) -> int:
        """Возвращает чистое время преобразования АЦП в МИЛЛИСЕКУНДАХ.
        Для QMC5883L время АЦП совпадает с периодом ODR, так как датчик
        не имеет отдельного времени простоя между измерениями."""
        raise NotImplementedError("QMC5883L не поддерживает отдельное время АЦП")