def write(chunk,width,height,length) -> None:
    buffer = bytearray()
    for y in range(height):
        for z in range(length):
            for x in range(width):
                id = chunk[int(x + z * width + y * width * length)]
                while ((id & -128) != 0): 
                    buffer.append(id & 127 | 128)
                    id >>= 7
                buffer.append(id)

    return bytearray(buffer)
