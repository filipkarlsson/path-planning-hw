def map_to_world_transform(x_map, y_map, map_w, map_h, world_w, world_h):
    xw = (x_map - map_w / 2) * world_w / map_w
    yw = -(y_map - map_h / 2) * world_h / map_h

    return (xw, yw)


print(map_to_world_transform(100, 100, 809, 689, 54, 58.7))
