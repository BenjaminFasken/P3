def modulo(x, _modulo):
    # To modulo a list, use numpy: x = np.array([126, 954, 1011, 627, 1]); _modulo = 400;
    return x - (x / _modulo).astype(int) * _modulo
