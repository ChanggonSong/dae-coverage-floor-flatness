# surface_profiling/utils/pcd_io.py

import os
import open3d as o3d
import numpy as np
import pandas as pd


def export_pcd_to_csv(pcd_path, csv_path):
    """
    PCD 파일을 읽어 x,y,z 형식으로 CSV 파일로 변환하여 저장.
    """
    if not os.path.exists(pcd_path):
        raise FileNotFoundError(f"[-] PCD file not found: {pcd_path}")

    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)  # shape: (N, 3)

    df = pd.DataFrame(points, columns=["x", "y", "z"])
    df.to_csv(csv_path, index=False)

    print(f"[+] PCD -> CSV convert complete: {csv_path} ({len(points)} points)")
    return csv_path