from math import sqrt

from march_shared_classes.exceptions.general_exceptions import IncorrectCoordinateError


class Vector3d(object):
    """A 3d vector class."""

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        z = self.z + other.z
        return Vector3d(x, y, z)

    def __sub__(self, other):
        x = self.x - other.x
        y = self.y - other.y
        z = self.z - other.z
        return Vector3d(x, y, z)

    def __div__(self, factor):
        x = self.x / factor
        y = self.y / factor
        z = self.z / factor
        return Vector3d(x, y, z)

    def __mul__(self, factor):
        x = self.x * factor
        y = self.y * factor
        z = self.z * factor
        return Vector3d(x, y, z)

    def __rmul__(self, factor):
        return self * factor

    def __getitem__(self, direction):
        return self.as_dictionary()[direction]

    def __iter__(self):
        for element in [self.x, self.y, self.z]:
            yield element

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.z == other.z:
            return True
        else:
            return False

    def __repr__(self):
        return 'x: {x}, y: {y}, z: {z}'.format(x=self.x, y=self.y, z=self.z)

    @classmethod
    def from_dictionary(cls, dic):
        """Creates a Vector3d object from a dictionary with keys 'x', 'y' and 'z'."""
        if {'x', 'y', 'z'} != set(dic.keys()):
            raise IncorrectCoordinateError()
        return cls(dic['x'], dic['y'], dic['z'])

    def as_dictionary(self):
        return{'x': self.x, 'y': self.y, 'z': self.z}

    def norm(self):
        return sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    @staticmethod
    def size():
        return 3

if __name__ == "__main__":
    vec = Vector3d(1,2,3)
    print(vec*3)
    print(vec*3.5)
    print(4 * vec)