
import os, scipy.io

fdir = "/home/jarvis/coneskf_fdi_ws/data/alfa/extracted/processed/carbonZ_2018-07-18-15-53-31_1_engine_failure"
mat_file = os.path.join(fdir, "carbonZ_2018-07-18-15-53-31_1_engine_failure.mat")

try:
    mat = scipy.io.loadmat(mat_file)
    print("MAT file keys:", [k for k in mat.keys() if not k.startswith("__")])
    for k in mat.keys():
        if not k.startswith("__"):
            val = mat[k]
            print(f"Key: {k}, type: {type(val)}")
            if hasattr(val, 'dtype') and val.dtype.names:
                print(f"  fields: {val.dtype.names}")
except Exception as e:
    print("MAT load error:", e)
