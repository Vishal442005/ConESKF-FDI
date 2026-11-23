
import os, scipy.io, numpy as np

fdir = "/home/jarvis/coneskf_fdi_ws/data/alfa/extracted/processed/carbonZ_2018-07-18-15-53-31_1_engine_failure"
mat_file = os.path.join(fdir, "carbonZ_2018-07-18-15-53-31_1_engine_failure.mat")

mat = scipy.io.loadmat(mat_file)
seq = mat['Sequence'][0, 0]
topics = [t[0] for t in seq['TopicNames'][0]]
print("Topic names count:", len(topics))
for t in topics:
    if 'fail' in t.lower() or 'engine' in t.lower() or 'status' in t.lower():
        print("  Topic:", t)

# Also check diagnostics or rc_out
for t in topics:
    if 'diag' in t.lower() or 'rc' in t.lower():
        print("  Control Topic:", t)
