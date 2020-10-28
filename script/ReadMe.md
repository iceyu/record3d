# Usage

1. 安装Open3D

    ```sh
    pip install open3d==0.11.1
    ```

2. 录制数据

    ```sh
    demo-main.py --path=E:/record3d/data --record=1
    ```

3. 转换为点云

    ```sh
    python save_pclouds.py --path=E:/record3d/data/2020-10-27-21-06-53
    ```

4. 查看点云

    ```sh
    python view_ply.py --file=E:\record3d\data\2020-10-27-21-06-53\ply\00000000.ply
    ```
