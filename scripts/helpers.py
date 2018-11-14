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


def contains_point(bound_x, bound_y, bound_w, bound_h, x, y):
    return (x >= bound_x) and (x < bound_x + bound_w) and (y >= bound_y) and (y < bound_y + bound_h)


def contains_box(bound_x, bound_y, bound_w, bound_h, x, y, w, h):
    top_l = (x >= bound_x) and (x < bound_x + bound_w) and (y >=
                                                            bound_y) and (y < bound_y + bound_h)
    top_r = (x + w >= bound_x) and (x + w < bound_x +
                                    bound_w) and (y >= bound_y) and (y < bound_y + bound_h)
    bottom_l = (x >= bound_x) and (x < bound_x + bound_w) and (y +
                                                               h >= bound_y) and (y + h < bound_y + bound_h)
    bottom_r = (x + w >= bound_x) and (x + w < bound_x +
                                       bound_w) and (y + h >= bound_y) and (y + h < bound_y + bound_h)

    return top_l and top_r and bottom_l and bottom_r


def map_to_world_transform(x_map, y_map, map_w, map_h, world_w, world_h):
    xw = (x_map - map_w / 2.0) * float(world_w) / map_w
    yw = (-y_map + map_h / 2.0) * float(world_h) / map_h

    return (xw, yw)


def world_to_map_transform(x_world, y_world, map_w, map_h, world_w, world_h):
    x_map = x_world * float(map_w) / world_w + map_w / 2.0
    y_map = (-y_world * float(map_h) / world_h + map_h / 2.0)

    return (x_map, y_map)


def transform_path(path, map_w, map_h, world_w, world_h):
    map_path = []
    for node in path:
        (xw, yw) = map_to_world_transform(
            node.x + node.w / 2, node.y + node.h / 2, map_w, map_h, world_w, world_h)
        map_path.append([xw, yw])

    return map_path
