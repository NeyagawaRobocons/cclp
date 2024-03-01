def translate_map(map, x, y):
    map_offseted = []
    for line in map:
        l = [[line[0][0]+x, line[0][1]+y], [line[1][0]+x, line[1][1]+y]]
        map_offseted.append(l)
    return map_offseted

def invert_x_map(map):
    map_inverted = []
    for line in map:
        l = [[-line[0][0], line[0][1]], [-line[1][0], line[1][1]]]
        map_inverted.append(l)
    return map_inverted

def print_map(map, name="Map"):
    print(f"{name} = [")
    for line in map:
        print(f"    [[{line[0][0]: > 7.3f}, {line[0][1]: > 7.3f}], [{line[1][0]: > 7.3f}, {line[1][1]: > 7.3f}]],")
    print("]")


left_map_origin_left = [
    [[0.0, 0.0], [3.424, 0.0]],
    [[3.424, 0.0], [3.424, 6.924]],
    [[3.424, 6.924], [0.0, 6.924]],
    [[0.0, 6.924], [0.0, 0.0]],
    [[1.019, 0.0], [1.019, 1.0]],
    [[0.8, 2.143], [3.424, 2.143]],
    [[0.0, 3.381], [1.286, 3.381]],
    [[0.8, 4.619], [2.105, 4.619]],
    [[2.105, 2.143], [2.105, 5.924]],
    [[1.019, 5.924], [1.019, 6.924]],
]

y_delta = 6.924 /2
x_delta = 3.424+0.038/2

left_map = translate_map(left_map_origin_left, -x_delta, -y_delta)
print_map(left_map, "left_map")
right_map = invert_x_map(left_map)
print_map(right_map, "right_map")
