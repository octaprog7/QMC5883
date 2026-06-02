"""Типы и вспомогательный код для интегральных магнитометров.
Магнитометры выдают значения в собственной трехмерной декартовой системе координат (x, y, z)."""
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

    def update(self, data: MagnetometerData):
        """
        Обновляет минимальные и максимальные значения на основе объекта MagnetometerData.
        Вызывай этот метод в цикле, пока вращаешь датчик "восьмеркой".
        """
        x, y, z = data.x, data.y, data.z

        if x < self.min_x: self.min_x = x
        if x > self.max_x: self.max_x = x

        if y < self.min_y: self.min_y = y
        if y > self.max_y: self.max_y = y

        if z < self.min_z: self.min_z = z
        if z > self.max_z: self.max_z = z

    def calculate_offsets(self) -> tuple:
        """
        Вычисляет вектор смещений (3 float).
        Вызывай этот метод ПОСЛЕ того, как собрал достаточно данных.
        """
        self.offset_x = 0.5 * (self.max_x + self.min_x)
        self.offset_y = 0.5 * (self.max_y + self.min_y)
        self.offset_z = 0.5 * (self.max_z + self.min_z)
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