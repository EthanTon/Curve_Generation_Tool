# Adapted from World Edit's VarIntIterator implementation: https://github.com/EngineHub/WorldEdit/blob/version/7.3.x/worldedit-core/src/main/java/com/sk89q/worldedit/internal/util/VarIntIterator.java

class VarIntIterator:
    def __init__(self, source: bytes):
        self.source = source
        self.index = 0
        self.has_next_int = False
        self.next_int = None

    def __iter__(self):
        return self

    def __next__(self) -> int:
        if not self.has_next():
            raise StopIteration
        self.has_next_int = False
        return self.next_int

    def has_next(self) -> bool:
        if self.has_next_int:
            return True

        if self.index >= len(self.source):
            return False

        self.next_int = self._read_next_int()
        self.has_next_int = True
        return True

    def _read_next_int(self) -> int:
        value = 0
        bits_read = 0

        while True:
            if self.index >= len(self.source):
                raise ValueError(
                    "Ran out of bytes while reading VarInt (probably corrupted data)"
                )

            next_byte = self.source[self.index]
            self.index += 1

            value |= (next_byte & 0x7F) << bits_read
            bits_read += 7

            if bits_read > 7 * 5:
                raise ValueError("VarInt too big (probably corrupted data)")

            if (next_byte & 0x80) == 0:
                break

        return value
