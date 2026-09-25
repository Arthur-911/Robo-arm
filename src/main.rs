use eframe::NativeOptions;
use kine_rs::ui::RoboSimApp;

fn main() -> eframe::Result<()> {
    tracing_subscriber::fmt::init();

    let native_options = NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1300.0, 840.0])
            .with_min_inner_size([960.0, 640.0])
            .with_title("kine-rs: High-Performance Robotic Arm Kinematics & IK Simulator"),
        ..Default::default()
    };

    eframe::run_native(
        "kine-rs",
        native_options,
        Box::new(|_cc| Ok(Box::new(RoboSimApp::default()))),
    )
}
