def maybe_cast_str_to_num(s):
    """
    Converts a string to an int if it is a plain number, or to a float if it contains a dot.
    
    Args:
        s (str): The string to be converted.
    
    Returns:
        int or float: The converted number, or the original string if conversion is not possible.
    """
    try:
        # Check if the string contains a dot, indicating it should be a float
        if '.' in s:
            return float(s)
        else:
            return int(s)
    except ValueError:
        # If conversion fails, return the original string
        return s