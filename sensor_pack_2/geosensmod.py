"""Типы и вспомогательный код для интегральных магнитометров.
Магнитометры выдают значения в собственной трехмерной декартовой системе координат (x, y, z)."""
import json
import micropython
from micropython import const
from collections import namedtuple

# Магнитометры практически всегда выдают значения в собственной трехмерной декартовой системе координат (x, y, z).
# Поле is_raw: bool Истина, когда поля кортежа содержат сырые(безразмерные) данные!
MagnetometerData = namedtuple("MagnetometerData", "x y z is_raw")
# Имена составляющих напряженности вектора магнитного поля в [Тесла]/[Гаусс]
Mag_Axis_Names = ('_', 'x', 'y', '_', 'z')
# Битовые маски осей (для флагов и комбинаций)
AXIS_X = const(1)  # 0b001
AXIS_Y = const(2)  # 0b010
AXIS_Z = const(4)  # 0b100
AXIS_ALL = const(AXIS_X | AXIS_Y | AXIS_Z) # 0b111 = 7

class MagRange:
    """Диапазоны измерений магнитного поля (в Гауссах, G).
    Используется как пространство имен для методов set_range_index."""
    G2 = const(0)   # ±2 G (высокая точность, QMC5883L)
    G8 = const(1)   # ±8 G (стандарт, QMC5883L, RM3100)
    G30 = const(2)  # ±30 G (широкий диапазон, MMC5603NJ)

def _axis_name_to_int(axis_name: str) -> int:
    """Преобразует имя оси ('x', 'y', 'z', 'X', 'Y', 'Z') в битовую маску оси: 1(X), 2(Y), 4(Z)"""
    if 1 != len(axis_name):
        raise ValueError(f"len of axis name: {axis_name}")
    an = axis_name.lower()
    if not an[0] in Mag_Axis_Names:
        raise ValueError(f"Invalid axis name: {axis_name}")
    return Mag_Axis_Names.index(an)


def check_axis_index(axis_index: int):
    """Проверяет числовой индекс оси. Он должен быть битовой маской: 1(X), 2(Y) или 4(Z)"""
    if axis_index not in (AXIS_X, AXIS_Y, AXIS_Z):
        raise ValueError(f"Invalid axis index: {axis_index}")

def axis_index_to_name(axis_index: int) -> str:
    """Преобразует битовую маску оси 1(x), 2(y), 4(z) в строку 'x', 'y', 'z'"""
    check_axis_index(axis_index)
    return Mag_Axis_Names[axis_index]

def axis_index_to_reg_addr(axis_index: int, offset: int, multiplier: int) -> int:
    """Преобразует битовую маску оси (1, 2, 4) в адрес регистра.
    Сдвиг >> 1 превращает 1->0, 2->1, 4->2 для корректного расчета адреса.
    """
    check_axis_index(axis_index)
    return offset + multiplier * (axis_index >> 1)

@micropython.native
def _get_min_max(value: float, current_min: float, current_max: float) -> tuple:
    """Возвращает экстремумы value в виде кортежа (current_min, current_max)."""
    if value < current_min:
        current_min = value
    elif value > current_max:
        current_max = value
    return current_min, current_max

@micropython.native
def _arith_mean(value_a: float, value_b: float) -> float:
    """Возвращает среднее арифметическое value_a и value_b."""
    return 0.5 * (value_a + value_b)

class HardIronCalibrator:
    """
    Калибратор магнитометра (компенсация жестких магнитных искажений (Hard Iron)),
    работающий напрямую с объектами MagnetometerData.
    Hard Iron (Жесткое железо) - Это влияние постоянных магнитов или сильно намагниченных ферромагнитных
    деталей рядом с датчиком (например, стальные винты крепления, динамики, моторы).
    Они создают постоянный вектор магнитного поля, который просто сдвигает центр измерений.
    Он не меняется, как бы вы ни вращали датчик.
    """

    def __init__(self):
        # Инициализация бесконечностями для правильного первого сравнения
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        self.min_z = float('inf')
        self.max_z = float('-inf')

        self._is_calibrated = False
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        # защита от математической ошибки nan (Not a Number — не число),
        # если пользователь или вызывающий код случайно вызовет метод calculate_offsets()
        self._has_data = False

    def update(self, data: MagnetometerData):
        """
        Обновляет минимальные и максимальные значения на основе объекта MagnetometerData.
        Вызывай этот метод в цикле, пока вращаешь датчик "восьмеркой".
        """
        self._has_data = True
        #
        self.min_x, self.max_x = _get_min_max(data.x, self.min_x, self.max_x)
        self.min_y, self.max_y = _get_min_max(data.y, self.min_y, self.max_y)
        self.min_z, self.max_z = _get_min_max(data.z, self.min_z, self.max_z)

    def calculate_offsets(self) -> tuple:
        """
        Вычисляет вектор смещений (3 float).
        Вызывай этот метод ПОСЛЕ того, как собрал достаточно данных.
        """
        if not self._has_data:
            return 0.0, 0.0, 0.0

        self.offset_x = _arith_mean(self.max_x, self.min_x)
        self.offset_y = _arith_mean(self.max_y, self.min_y)
        self.offset_z = _arith_mean(self.max_z, self.min_z)
        self._is_calibrated = True

        return self.offset_x, self.offset_y, self.offset_z

    def apply(self, data: MagnetometerData) -> MagnetometerData:
        """
        Применяет калибровку к объекту MagnetometerData.
        Возвращает НОВЫЙ объект MagnetometerData с откалиброванными значениями.
        """
        if not self._is_calibrated:
            # Если калибровка не проведена, возвращаем исходный объект без изменений
            return data

        return MagnetometerData(
            x=data.x - self.offset_x,
            y=data.y - self.offset_y,
            z=data.z - self.offset_z,
            is_raw=data.is_raw  # сохраняю флаг формата данных!
        )

    def is_calibrated(self) -> bool:
        return self._is_calibrated


def save_calibration(offsets: tuple, filename: str = "mag_calib.json") -> bool:
    """Сохраняет кортеж смещений (offset_x, offset_y, offset_z) в файл JSON.
    Возвращает True в случае успеха."""
    try:
        with open(filename, "w") as f:
            json.dump(offsets, f)
        return True
    except (OSError, IndexError):
        return False

def load_calibration(filename: str = "mag_calib.json") -> tuple or None:
    """Загружает смещения из JSON файла.
    Возвращает кортеж (offset_x, offset_y, offset_z) или None, если файл поврежден/отсутствует."""
    try:
        with open(filename, "r") as f:
            data = json.load(f)  # Считает список Python

        # проверяю, что массив имеет нужную длину
        if isinstance(data, list) and len(data) == 3:
            return data[0], data[1], data[2]
        return None
    except (OSError, ValueError):
        return None