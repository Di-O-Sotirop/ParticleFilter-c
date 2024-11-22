#!/usr/bin/env python

import array

with open('odom_expo_2.csv', 'rt') as f:
    text = f.read()
    entries = text.split(',')
    values = [int(x) for x in entries]
    # do a scalar here: if your input goes from [-100, 100] then
    #   you may need to translate/scale into [0, 2^16-1] for
    #   16-bit PCM
    # e.g.:
    #   values = [(val * scale) for val in values]

with open('output.pcm', 'wb') as out:
    pcm_vals = array.array('h', values) # 16-bit signed
    pcm_vals.tofile(out)