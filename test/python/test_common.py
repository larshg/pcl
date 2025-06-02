# for pytest
import pcl_common_ext

def test_pointcloud():
    cloud = pcl_common_ext.PointCloudXYZ()
    cloud.resize(5)
    assert len(cloud) == 5
    cloud[0].y = 1.0
    assert cloud[0].y == 1.0
    # TODO expand
