use kine_rs::presets::{SAMPLE_URDF_INDUSTRIAL_6DOF, SAMPLE_URDF_PLANAR_3DOF};
use kine_rs::urdf::{export_urdf, parse_urdf};

#[test]
fn test_parse_planar_urdf() {
    let robot = parse_urdf(SAMPLE_URDF_PLANAR_3DOF).expect("Should parse planar URDF successfully");
    assert_eq!(robot.name, "planar_3dof_arm");
    assert_eq!(robot.joints.len(), 3);
    assert_eq!(robot.links.len(), 3);
}

#[test]
fn test_parse_industrial_urdf() {
    let robot = parse_urdf(SAMPLE_URDF_INDUSTRIAL_6DOF)
        .expect("Should parse 6-DOF industrial URDF successfully");
    assert_eq!(robot.name, "industrial_6dof_robot");
    assert_eq!(robot.joints.len(), 6);
    assert_eq!(robot.dof(), 6);
}

#[test]
fn test_export_and_reimport_urdf() {
    let original = parse_urdf(SAMPLE_URDF_PLANAR_3DOF).unwrap();
    let exported_xml = export_urdf(&original);

    let reimported = parse_urdf(&exported_xml).expect("Should parse re-exported URDF successfully");
    assert_eq!(reimported.joints.len(), original.joints.len());
    assert_eq!(reimported.dof(), original.dof());
}
