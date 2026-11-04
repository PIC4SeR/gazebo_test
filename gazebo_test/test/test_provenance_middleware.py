import yaml

from gazebo_test.utils.provenance import write_experiment_provenance


def test_provenance_records_the_middleware(tmp_path, monkeypatch):
    monkeypatch.setenv("CYCLONEDDS_URI", "file:///cfg/cyclonedds.xml")
    monkeypatch.setenv("ROS_DOMAIN_ID", "17")
    path = write_experiment_provenance(base_path=tmp_path, algorithm_name="a",
                                       experiment_identifier="e")
    middleware = yaml.safe_load(path.read_text())["middleware"]
    assert middleware["cyclonedds_uri"] == "file:///cfg/cyclonedds.xml"
    assert middleware["ros_domain_id"] == "17"
    assert middleware["rmw_implementation"]  # rmw_fastrtps_cpp unless RMW_IMPLEMENTATION is set
