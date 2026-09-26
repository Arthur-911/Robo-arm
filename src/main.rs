#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result<()> {
    let _ = tracing_subscriber::fmt::try_init();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1300.0, 840.0])
            .with_min_inner_size([960.0, 640.0])
            .with_title("kine-rs: High-Performance Robotic Arm Kinematics & IK Simulator"),
        ..Default::default()
    };

    eframe::run_native(
        "kine-rs",
        native_options,
        Box::new(|_cc| Ok(Box::new(kine_rs::ui::RoboSimApp::default()))),
    )
}

#[cfg(target_arch = "wasm32")]
fn main() {
    let _ = kine_rs::web::start();
}
