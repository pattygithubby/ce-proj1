def normalize_colors(red16bit, green16bit, blue16bit):
    """Normaliserer 16-bit RGB-værdier til 0-255 skala."""
    red = (red16bit / 65535) * 255
    green = (green16bit / 65535) * 255
    blue = (blue16bit / 65535) * 255

    total_sum = red + green + blue
    if total_sum == 0:
        return 0, 0, 0  # Undgå division med nul

    norm_red = red / total_sum
    norm_green = green / total_sum
    norm_blue = blue / total_sum

    return norm_red, norm_green, norm_blue
