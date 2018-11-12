# returns True if there is no pixel in square(x,x+w,y,y+h) with value != 1
# Pixel_array is 1-D flat array.


def free_space(x, y, w, h, pixel_array, img_width):
    for v in range(y, y + h):
        for u in range(x, x + w):
            if pixel_array[u + v * img_width] == 0:
                return False

    return True


def mixed_space(x, y, w, h, pixel_array, img_width):
    free_pixels = filled_pixels = 0
    for v in range(y, y + h):
        for u in range(x, x + w):
            if pixel_array[u + v * img_width] == 0:
                filled_pixels = filled_pixels + 1
            else:
                free_pixels = free_pixels + 1

            if (free_pixels > 0) and (filled_pixels > 0):
                return True

    return (free_pixels > 0) and (filled_pixels > 0)


def frange(start, stop, step):
    i = start
    while i < stop:
        yield i
        i += step
