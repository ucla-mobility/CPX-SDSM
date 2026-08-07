"""Small pure-Python geohash helper used for ETX Regional subscriptions."""

_B32 = "0123456789bcdefghjkmnpqrstuvwxyz"


def encode(lat: float, lon: float, precision: int = 12) -> str:
    lat_range = [-90.0, 90.0]
    lon_range = [-180.0, 180.0]
    out: list[str] = []
    bit = 0
    ch = 0
    even = True

    while len(out) < precision:
        if even:
            mid = (lon_range[0] + lon_range[1]) / 2
            if lon > mid:
                ch = (ch << 1) | 1
                lon_range[0] = mid
            else:
                ch <<= 1
                lon_range[1] = mid
        else:
            mid = (lat_range[0] + lat_range[1]) / 2
            if lat > mid:
                ch = (ch << 1) | 1
                lat_range[0] = mid
            else:
                ch <<= 1
                lat_range[1] = mid

        even = not even
        bit += 1
        if bit == 5:
            out.append(_B32[ch])
            bit = 0
            ch = 0

    return "".join(out)


def decode_exactly(ghash: str) -> tuple[float, float, float, float]:
    lat_range = [-90.0, 90.0]
    lon_range = [-180.0, 180.0]
    even = True

    for char in ghash:
        code = _B32.index(char)
        for mask in (16, 8, 4, 2, 1):
            if even:
                mid = (lon_range[0] + lon_range[1]) / 2
                if code & mask:
                    lon_range[0] = mid
                else:
                    lon_range[1] = mid
            else:
                mid = (lat_range[0] + lat_range[1]) / 2
                if code & mask:
                    lat_range[0] = mid
                else:
                    lat_range[1] = mid
            even = not even

    lat = (lat_range[0] + lat_range[1]) / 2
    lon = (lon_range[0] + lon_range[1]) / 2
    return lat, lon, lat_range[1] - lat, lon_range[1] - lon


def neighbors(ghash: str) -> list[str]:
    lat, lon, dlat, dlon = decode_exactly(ghash)
    precision = len(ghash)
    out: list[str] = []
    for d_lat in (dlat * 2, 0.0, -dlat * 2):
        for d_lon in (-dlon * 2, 0.0, dlon * 2):
            if d_lat == 0.0 and d_lon == 0.0:
                continue
            out.append(encode(lat + d_lat, lon + d_lon, precision))
    return out
