def weighted_average(base_value, other_value, parameter):
    """Compute the weighted average of two values with normalised weight parameter.

    :param base_value: The first value for the weighted average, return this if parameter is 0
    :param other_value: The second value for the weighted average, return this if parameter is 1
    :param: parameter: The normalised weight parameter, the parameter that deterines the weight of the second value

    :return: A value which is the weighted average of the given values
    """
    return base_value * (1 - parameter) + other_value * parameter


def merge_dictionaries(dic_one, dic_two):
    """Combines the key value pairs of two dicitonaries into a new dicionary.

    Throws an error when both dictionaries contain the same key and the corresponding values are not equal.

    :param dic_one: One of the dictionaries which is to be merged
    :param dic_two: One of the dictionaries which is to be merged

    :return: The merged dictionary, has the same key value pairs as the pairs in the given dictionaries combined
    """
    merged_dic = {}
    for key_one in dic_one:
        if key_one not in dic_two or dic_one[key_one] == dic_two[key_one]:
            merged_dic[key_one] = dic_one[key_one]
        else:
            raise KeyError('Dictionaries to be merged both contain key {key} with differing values'.
                           format(key=key_one))
    for key_two in dic_two:
        if key_two not in dic_one or dic_one[key_two] == dic_two[key_two]:
            merged_dic[key_two] = dic_two[key_two]
        else:
            raise KeyError('Dictionaries to be merged both contain key {key} with differing values'.
                           format(key=key_two))
    return merged_dic
