#!/usr/bin/env python3
from numpy import cumsum, insert, append, shape, delete


def running_mean(old_array, new_value):
    def calculate_mean(x, N):
        cum_sum = cumsum(insert(x, 0, 0))
        return (cum_sum[N:] - cum_sum[:-N]) / float(N)

    new_array = append(delete(old_array, 0), new_value)
    new_mean = calculate_mean(new_array, shape(new_array)[0])[0]
    return new_array, new_mean
