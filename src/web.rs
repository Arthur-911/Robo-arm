#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(start)]
pub fn start() -> Result<(), JsValue> {
    console_error_panic_hook::set_once();
    tracing_subscriber::fmt::init();

    wasm_bindgen_futures::spawn_local(async {
        use wasm_bindgen::JsCast;

        let web_options = eframe::WebOptions::default();
        let document = web_sys::window()
            .and_then(|win| win.document())
            .expect("Failed to get DOM document");
        let canvas = document
            .get_element_by_id("kine_canvas")
            .expect("Failed to locate canvas element #kine_canvas")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("Element #kine_canvas is not a HTMLCanvasElement");

        eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|_cc| Ok(Box::new(crate::ui::RoboSimApp::default()))),
            )
            .await
            .expect("Failed to launch eframe WebRunner");
    });

    Ok(())
}
